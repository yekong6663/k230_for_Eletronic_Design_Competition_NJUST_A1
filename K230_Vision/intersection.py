"""
路口检测模块 (备用独立模块, 通常融入 lane_detect 运行)

算法管线:
  ROI 上半区域灰度行均值扫描 → 横向黑线检测
  → 类型判定(十字/T字左/T字右) → 多帧确认

依赖:
  - lane_detect.last_left_coeff / last_right_coeff (类型判定用)
  - lane_detect.last_warped (鸟瞰图 Sobel 用, 暂注释)

帧格式 (0x02):
  | 0xAA | 0x02 | intersection_type(1B) | direction(1B) | XOR |
  - type: 0=无, 1=十字, 2=T字左, 3=T字右
  - direction: 0=无, 1=左转, 2=右转, 3=直行
"""

import config
from detector_base import Detector


class IntersectionDetector(Detector):
    """路口检测器，继承自 Detector 基类"""

    CONFIRM_FRAMES  = 5        # 多帧确认帧数
    MIN_DARK_RUN    = 3        # 最少连续暗行数
    DARK_RATIO      = 0.55     # 行均值低于 baseline * DARK_RATIO → 暗行

    def __init__(self):
        super().__init__(name="Intersection")

        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        super().Init()
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0

    def Process(self, img, left_coeff, right_coeff, warped=None):
        """
        扫描 ROI 上半区域寻找横向黑线

        参数:
            img         : 原始图像 (RGB565)
            left_coeff  : 左车道线拟合系数 (a,b,c) or None
            right_coeff : 右车道线拟合系数 (a,b,c) or None
            warped      : 鸟瞰图 (可选, Sobel 验证用, 暂不使用)

        返回:
            (type, direction) or None
        """
        if left_coeff is None or right_coeff is None:
            self._ResetConfirm()
            return None

        # 二次拟合: x = a·y² + b·y + c
        _a_l, b_l, _c_l = left_coeff
        _a_r, b_r, _c_r = right_coeff

        # 直线回归: x = b·y + c  (切换时解除注释)
        # _a_l, b_l, _c_l = 0.0, left_coeff[1], left_coeff[2]
        # _a_r, b_r, _c_r = 0.0, right_coeff[1], right_coeff[2]

        # ---- ① 灰度行均值扫描 ----
        gray = self._ExtractRoiGray(img)
        row_means = self._ScanRowMeans(gray)

        if len(row_means) == 0:
            self._ResetConfirm()
            return None

        # ---- ② 寻找连续暗行 (黑线带) ----
        max_dark_run = self._FindMaxDarkRun(row_means)

        # ---- ③ 类型判定 ----
        itype = self._Classify(max_dark_run, b_l, b_r)

        # ---- 鸟瞰图 Sobel 横向边缘 (暂注释) ----
        # if itype > 0 and warped is not None:
        #     if self._SobelHorizontalScore(warped) < 0.05:
        #         itype = 0

        # ---- ④ 多帧确认 ----
        return self._ConfirmAndReport(itype)

    def Reset(self):
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0

    # ================================================================
    # 私有方法 — 灰度提取 & 扫描
    # ================================================================

    @staticmethod
    def _ExtractRoiGray(img):
        """从原始图提取 ROI 灰度图"""
        roi_x = 0
        roi_y = int(config.IMAGE_HEIGHT * config.ROI_Y_START_RATIO)
        roi_w = config.IMAGE_WIDTH
        roi_h = (int(config.IMAGE_HEIGHT * config.ROI_Y_END_RATIO)
                 - roi_y)

        roi = img.copy(roi=(roi_x, roi_y, roi_w, roi_h))
        roi.gaussian(3)              # 抑制传感器噪声, 避免误判暗行
        return roi.to_grayscale()

    @staticmethod
    def _ScanRowMeans(gray):
        """扫描灰度图上半 1/2 区域, 逐行计算平均灰度"""
        w = gray.width()
        h = gray.height()
        scan_end = h // 2

        row_means = []
        for y in range(scan_end):
            row_sum = 0
            for x in range(w):
                row_sum += gray.get_pixel(x, y)
            row_means.append(row_sum / w)
        return row_means

    def _FindMaxDarkRun(self, row_means):
        """在行均值序列中寻找最长连续暗行段 (游程编码)"""
        baseline = sum(row_means) / len(row_means)
        threshold = baseline * self.DARK_RATIO

        in_dark = False
        dark_start = 0
        max_dark_run = 0

        for y, mean in enumerate(row_means):
            if mean < threshold:
                if not in_dark:
                    in_dark = True
                    dark_start = y
            else:
                if in_dark:
                    dark_run = y - dark_start
                    if dark_run > max_dark_run:
                        max_dark_run = dark_run
                    in_dark = False

        if in_dark:
            dark_run = len(row_means) - dark_start
            if dark_run > max_dark_run:
                max_dark_run = dark_run

        return max_dark_run

    # ================================================================
    # 私有方法 — 分类 & 确认
    # ================================================================

    def _Classify(self, max_dark_run, b_l, b_r):
        """
        根据暗行长度和车道线斜率判定路口类型

        返回: 0=无, 1=十字, 2=T字左, 3=T字右
        """
        if max_dark_run < self.MIN_DARK_RUN:
            return 0

        slope_diff = abs(b_r - b_l)
        if slope_diff < 0.2:
            return 1            # 十字
        if b_r > b_l:
            return 2            # T字左
        return 3                # T字右

    def _ResetConfirm(self):
        self._confirm_count = 0
        self._last_type = 0

    def _ConfirmAndReport(self, itype):
        """
        多帧确认 & 上报

        返回: (type, direction) or None
        """
        if itype > 0 and itype == self._last_type:
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        self._last_type = itype

        if self._confirm_count >= self.CONFIRM_FRAMES:
            # type→direction: 十字→直行(3), T字左→左转(1), T字右→右转(2)
            direction = {1: 3, 2: 1, 3: 2}.get(itype, 0)
            self._result = {"type": itype, "direction": direction}
            self._confirm_count = 0
            return (itype, direction)

        return None

    # ---- 鸟瞰图 Sobel 横向边缘检测 (暂不使用, 需要时解除注释) ----

    # def _SobelHorizontalScore(self, warped):
    #     """
    #     对鸟瞰图上方区域做横向 Sobel 边缘投影, 返回归一化得分
    #
    #     横边密集 → 存在交叉道路 → 得分高 → 与灰度行均值联合判定
    #     """
    #     w = warped.width()
    #     h = warped.height()
    #     y_end = h // 3
    #
    #     edge_sum = 0
    #     count = 0
    #     for y in range(1, y_end - 1):
    #         for x in range(1, w - 1):
    #             gx = (
    #                 -1 * warped.get_pixel(x - 1, y - 1)
    #                 + 1 * warped.get_pixel(x + 1, y - 1)
    #                 - 2 * warped.get_pixel(x - 1, y)
    #                 + 2 * warped.get_pixel(x + 1, y)
    #                 - 1 * warped.get_pixel(x - 1, y + 1)
    #                 + 1 * warped.get_pixel(x + 1, y + 1)
    #             )
    #             edge_sum += abs(gx)
    #             count += 1
    #
    #     if count == 0:
    #         return 0.0
    #     return edge_sum / (count * 255.0)
