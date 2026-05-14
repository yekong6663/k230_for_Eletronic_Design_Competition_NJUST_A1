"""
双线车道线检测模块

车道线始终为双线 (车跑在车道中间):
  - 双线有效: 左右车道线同时可见 → 精确 offset + angle
  - 丢线: 任一侧点数不足 → valid=0

算法管线:
  原始图 → ROI裁剪 → 高斯滤波 → 转灰度 → Canny边缘检测 → 透视变换(鸟瞰)
  → 逐行扫描提取左右点云 → 二次曲线拟合 → 计算offset/angle/valid
"""

import math
import config
from calib_data import homography
from detector_base import Detector


class LaneDetector(Detector):
    """双线车道线检测器，继承自 Detector 基类"""

    def __init__(self):
        # 调用基类构造，设置名称
        super().__init__(name="LaneDetector")

        # 连续丢线的帧数计数器
        # 双线有效时清零，丢线时累加
        self._consecutive_invalid = 0

        # 最新结果缓存
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}

        # ---- 供路口检测模块 / 可视化复用的中间结果 ----
        # 左车道线二次拟合系数 (a, b, c)，拟合失败时为 None
        self.last_left_coeff = None
        # 右车道线二次拟合系数 (a, b, c)，拟合失败时为 None
        self.last_right_coeff = None
        # 最近一帧的鸟瞰图 (透视变换后的 Image 对象)
        self.last_warped = None
        # 左车道线提取的点云 [(x, y), ...]
        self.last_left_pts = []
        # 右车道线提取的点云 [(x, y), ...]
        self.last_right_pts = []

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        """初始化车道线检测器内部状态"""
        super().Init()
        self._consecutive_invalid = 0
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
        self.last_left_coeff  = None
        self.last_right_coeff = None
        self.last_warped      = None
        self.last_left_pts    = []
        self.last_right_pts   = []

    def Process(self, img):
        """
        对一帧图像执行完整的车道线检测管线

        参数:
            img: OpenMV image 对象 (RGB565, QVGA)

        返回:
            dict: {"offset": float, "angle": float, "valid": bool}
        """
        # ---- ① 预处理: ROI → 高斯 → 灰度 → Canny ----
        preprocessed = self._Preprocess(img)
        if preprocessed is None:
            self._HandleInvalid()
            return self._result

        # ---- ② 透视变换: 斜视图 → 鸟瞰俯视图 ----
        warped = self._WarpPerspective(preprocessed)
        self.last_warped = warped

        # ---- ③ 逐行扫描提取左右车道线点云 ----
        left_pts, right_pts = self._ExtractLanePoints(warped)
        self.last_left_pts  = left_pts
        self.last_right_pts = right_pts

        # ---- ④ 二次曲线拟合 ----
        left_coeff  = self._FitQuadratic(left_pts)
        right_coeff = self._FitQuadratic(right_pts)

        self.last_left_coeff  = left_coeff
        self.last_right_coeff = right_coeff

        # ---- ⑤ 计算 offset 与 angle ----
        if left_coeff is not None and right_coeff is not None:
            offset, angle = self._CalcDualLane(
                left_coeff, right_coeff, warped.width()
            )
            valid = True
        else:
            offset, angle, valid = 0.0, 0.0, False

        # ---- ⑥ 丢线防抖 ----
        if valid:
            self._consecutive_invalid = 0
        else:
            self._consecutive_invalid += 1
            if self._consecutive_invalid < config.MAX_CONSECUTIVE_INVALID:
                self._result["valid"] = False
                return self._result

        self._result = {"offset": offset, "angle": angle, "valid": valid}
        return self._result

    def Reset(self):
        """重置车道线检测器: 清空计数器与中间结果"""
        self._consecutive_invalid = 0
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
        self.last_left_coeff  = None
        self.last_right_coeff = None
        self.last_warped      = None
        self.last_left_pts    = []
        self.last_right_pts   = []

    # ================================================================
    # 预处理管线
    # ================================================================

    def _Preprocess(self, img):
        """
        ROI 裁剪 → 高斯滤波 → 转灰度 → Canny 边缘检测

        参数:
            img: 原始图像 (RGB565, QVGA)

        返回:
            Image: Canny 边缘图 (单通道, 255=边缘, 0=背景)
            None:   ROI 区域不合法
        """
        # ROI 裁剪: x 全宽, y 取 40%~95%
        x_start = 0
        y_start = int(config.IMAGE_HEIGHT * config.ROI_Y_START_RATIO)
        w       = config.IMAGE_WIDTH
        h       = int(config.IMAGE_HEIGHT * config.ROI_Y_END_RATIO) - y_start

        if h <= 0:
            return None

        roi = img.copy(roi=(x_start, y_start, w, h))
        roi.gaussian(config.GAUSSIAN_KERNEL)
        gray = roi.to_grayscale()
        return gray.find_edges(
            image.EDGE_CANNY,
            threshold=(config.CANNY_LOW, config.CANNY_HIGH)
        )

    def _WarpPerspective(self, img):
        """透视变换 → 鸟瞰图 (标定数据未填入时退化为原图)"""
        try:
            return img.warp_perspective(homography.H_MATRIX)
        except Exception:
            return img

    # ================================================================
    # 点云提取 & 拟合
    # ================================================================

    def _ExtractLanePoints(self, warped):
        """
        逐行扫描鸟瞰图，提取左右车道线边缘点云

        每行取最左、最右边缘像素，按图像中线分左右。
        """
        w = warped.width()
        h = warped.height()
        mid_x = w // 2

        left_pts  = []
        right_pts = []

        for y in range(h):
            edge_xs = []
            for x in range(w):
                if warped.get_pixel(x, y) > 0:
                    edge_xs.append(x)

            if len(edge_xs) == 0:
                continue

            leftmost  = edge_xs[0]
            rightmost = edge_xs[-1]

            if leftmost < mid_x:
                left_pts.append((leftmost, y))
            if rightmost > mid_x:
                right_pts.append((rightmost, y))

        return left_pts, right_pts

    def _FitQuadratic(self, points):
        """
        最小二乘拟合 x = a·y² + b·y + c

        求解方法: 克莱姆法则解 3×3 正规方程。
        返回 (a, b, c) 或 None (点数不足 / 行列式为零 / 曲率异常)。
        """
        if len(points) < config.FIT_MIN_POINTS:
            return None

        n = len(points)
        sum_y = sum_y2 = sum_y3 = sum_y4 = 0.0
        sum_x = sum_xy = sum_xy2 = 0.0

        for x, y_coord in points:
            y_f = float(y_coord)
            y2  = y_f * y_f
            sum_y   += y_f
            sum_y2  += y2
            sum_y3  += y2 * y_f
            sum_y4  += y2 * y2
            sum_x   += x
            sum_xy  += x * y_f
            sum_xy2 += x * y2

        # 系数矩阵行列式 (按第一行展开)
        det = (
              n       * (sum_y2 * sum_y4 - sum_y3 * sum_y3)
            - sum_y   * (sum_y  * sum_y4 - sum_y3 * sum_y2)
            + sum_y2  * (sum_y  * sum_y3 - sum_y2 * sum_y2)
        )

        if abs(det) < 1e-12:
            return None

        inv_det = 1.0 / det

        # 克莱姆法则求解 c (截距)
        c = inv_det * (
              sum_x   * (sum_y2 * sum_y4 - sum_y3 * sum_y3)
            - sum_y   * (sum_xy  * sum_y4 - sum_xy2 * sum_y3)
            + sum_y2  * (sum_xy  * sum_y3 - sum_xy2 * sum_y2)
        )

        # 克莱姆法则求解 b (斜率)
        b = inv_det * (
              n       * (sum_xy  * sum_y4 - sum_xy2 * sum_y3)
            - sum_x   * (sum_y   * sum_y4 - sum_y3  * sum_y2)
            + sum_y2  * (sum_y   * sum_xy2 - sum_xy * sum_y3)
        )

        # 克莱姆法则求解 a (曲率)
        a = inv_det * (
              n       * (sum_y2 * sum_xy2 - sum_y3 * sum_xy)
            - sum_y   * (sum_y  * sum_xy2 - sum_xy * sum_y3)
            + sum_x   * (sum_y  * sum_y3  - sum_y2 * sum_y2)
        )

        if abs(a) > 0.005:
            return None
        return (a, b, c)

    # ================================================================
    # 偏差计算
    # ================================================================

    def _CalcDualLane(self, left_coeff, right_coeff, img_w):
        """
        双线模式: 用左右车道线的拟合系数计算 offset 与 angle

        offset = (cL + cR) / 2 - 图像中心  → 转为 mm
        angle  = atan((bL + bR) / 2)        → 转为 °
        """
        a_l, b_l, c_l = left_coeff
        a_r, b_r, c_r = right_coeff

        lane_center_px = (c_l + c_r) / 2.0
        image_center_px = img_w / 2.0
        offset_px = lane_center_px - image_center_px

        offset_mm = offset_px * config.MM_PER_PIXEL
        angle_rad = math.atan((b_l + b_r) / 2.0)
        angle_deg = math.degrees(angle_rad)

        return offset_mm, angle_deg

    # ================================================================
    # 丢线处理
    # ================================================================

    def _HandleInvalid(self):
        """预处理失败时的丢线处理: 累加丢线计数，清除中间结果"""
        self._consecutive_invalid += 1
        self.last_left_coeff  = None
        self.last_right_coeff = None
        self.last_left_pts    = []
        self.last_right_pts   = []
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
