"""
双线车道线检测模块 (8邻域边缘追踪算法)

算法管线:
  灰度图 → 直方图二值化 → 底部种子点搜索 → 8邻域边缘追踪
  → 中线点计算 → 最小二乘直线拟合 → 计算offset/angle

车道线始终为双线 (车跑在车道中间):
  - 双线有效: 左右车道线同时可见 → 精确 offset + angle
  - 丢线: 任一侧点数不足 → valid=0
"""

import math
import config
from detector_base import Detector

# 八邻域方向偏移
SEEDS_L = [(-1, 0), (-1, -1), (0, -1), (1, -1),
           (1, 0), (1, 1), (0, 1), (-1, 1)]
SEEDS_R = [(1, 0), (1, -1), (0, -1), (-1, -1),
           (-1, 0), (-1, 1), (0, 1), (1, 1)]


def _safe_pixel(img, x, y, default=0):
    """安全取像素，防越界和 None"""
    if 0 <= x < img.width() and 0 <= y < img.height():
        val = img.get_pixel(x, y)
        return val if val is not None else default
    return default


class LaneDetector(Detector):
    """双线车道线检测器，8邻域边缘追踪"""

    SEARCH_DEPTH_RATIO = 2.0 / 3.0   # 从底部向上搜索的图像比例

    def __init__(self):
        super().__init__(name="LaneDetector")
        self._consecutive_invalid = 0
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
        self.last_left_pts   = []
        self.last_right_pts  = []
        self.last_mid_pts    = []
        self.last_k          = 0.0
        self.last_b          = 0.0

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        super().Init()
        self._consecutive_invalid = 0
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
        self.last_left_pts   = []
        self.last_right_pts  = []
        self.last_mid_pts    = []
        self.last_k          = 0.0
        self.last_b          = 0.0

    def Process(self, img):
        """
        对一帧灰度图像执行车道线检测 (原地二值化)

        参数:
            img: 灰度图 (GRAYSCALE), 会被原地二值化修改

        返回: {"offset": float, "angle": float, "valid": bool}
        """
        w = img.width()
        h = img.height()

        # ---- ① 直方图二值化 ----
        hist = img.get_histogram()
        thresh = hist.get_threshold()
        img.binary([(thresh.value(), 255)], invert=False)

        # ---- ② 搜索左右车道线边缘点 ----
        left_pts, right_pts = self._SearchLine(img, w // 2)
        self.last_left_pts  = left_pts
        self.last_right_pts = right_pts

        if len(left_pts) < 3 or len(right_pts) < 3:
            self._HandleInvalid()
            return self._result

        # ---- ③ 计算中线点 ----
        mid_pts = []
        for l, r in zip(left_pts, right_pts):
            mid_x = (l[0] + r[0]) // 2
            mid_y = (l[1] + r[1]) // 2
            mid_pts.append((mid_x, mid_y))
        self.last_mid_pts = mid_pts

        # ---- ④ 最小二乘直线拟合 x = k·y + b ----
        k, b = self._FitMidLine(mid_pts)
        self.last_k = k
        self.last_b = b

        # ---- ⑤ 计算 offset & angle ----
        y_near = float(h - 1)
        x_mid = k * y_near + b
        offset_px = x_mid - w / 2.0
        offset_mm = offset_px * config.MM_PER_PIXEL
        angle_deg = math.degrees(math.atan(k))

        self._consecutive_invalid = 0
        self._result = {"offset": offset_mm, "angle": angle_deg, "valid": True}
        return self._result

    def Reset(self):
        self._consecutive_invalid = 0
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
        self.last_left_pts   = []
        self.last_right_pts  = []
        self.last_mid_pts    = []
        self.last_k          = 0.0
        self.last_b          = 0.0

    def SendToUart(self):
        """发送车道线检测结果"""
        if self._uart is not None:
            self._uart.SendLane(
                self._result["offset"],
                self._result["angle"],
                self._result["valid"]
            )
            print("[TX] Lane | offset=%+.1fpx  angle=%+.1fdeg  valid=%d"
                  % (self._result["offset"], self._result["angle"],
                     self._result["valid"]))

    # ================================================================
    # 8邻域边缘追踪
    # ================================================================

    def _SearchLine(self, img, mid_x):
        """从图像底部向上搜索左右车道线边缘"""
        w = img.width()
        h = img.height()
        start_y = h - 1

        # ---- 底部种子点搜索 ----
        start_l = [0, start_y]
        start_r = [w - 1, start_y]

        mid_pixel = _safe_pixel(img, mid_x, start_y)
        if mid_pixel > 128:
            # 中点白色(在车道内)，向两侧找黑边(车道线边缘)
            for x in range(mid_x, 0, -1):
                if (_safe_pixel(img, x, start_y) < 128 and
                        _safe_pixel(img, x - 1, start_y) < 128):
                    start_l = [x, start_y]
                    break
            for x in range(mid_x, w - 1):
                if (_safe_pixel(img, x, start_y) < 128 and
                        _safe_pixel(img, x + 1, start_y) < 128):
                    start_r = [x, start_y]
                    break
        else:
            # 中点黑色(压在线上)，向两侧找白边定位
            found = False
            for d in range(1, w // 2):
                if _safe_pixel(img, mid_x - d, start_y) > 128:
                    start_r = [mid_x - d + 1, start_y]
                    for x in range(start_r[0], 0, -1):
                        if (_safe_pixel(img, x, start_y) < 128 and
                                _safe_pixel(img, x - 1, start_y) < 128):
                            start_l = [x, start_y]
                            break
                    found = True
                    break
                elif _safe_pixel(img, mid_x + d, start_y) > 128:
                    start_l = [mid_x + d - 1, start_y]
                    for x in range(start_l[0], w - 1):
                        if (_safe_pixel(img, x, start_y) < 128 and
                                _safe_pixel(img, x + 1, start_y) < 128):
                            start_r = [x, start_y]
                            break
                    found = True
                    break
            if not found:
                return [], []

        # ---- 逐行向上追踪 ----
        pts_l = [start_l[:]]
        pts_r = [start_r[:]]
        cp_l  = start_l[:]
        cp_r  = start_r[:]

        max_iter = int(h * self.SEARCH_DEPTH_RATIO)

        for _ in range(max_iter):
            # 左边缘邻域
            temp_l = []
            search_l = [[cp_l[0] + dx, cp_l[1] + dy] for dx, dy in SEEDS_L]
            for i in range(8):
                x1, y1 = search_l[i]
                x2, y2 = search_l[(i + 1) & 7]
                if (_safe_pixel(img, x1, y1) < 128 and
                        _safe_pixel(img, x2, y2) > 128):
                    temp_l.append([x1, y1])
            if temp_l:
                cp_l = min(temp_l, key=lambda p: p[1])
                pts_l.append(cp_l[:])

            # 右边缘邻域
            temp_r = []
            search_r = [[cp_r[0] + dx, cp_r[1] + dy] for dx, dy in SEEDS_R]
            for i in range(8):
                x1, y1 = search_r[i]
                x2, y2 = search_r[(i + 1) & 7]
                if (_safe_pixel(img, x1, y1) < 128 and
                        _safe_pixel(img, x2, y2) > 128):
                    temp_r.append([x1, y1])
            if temp_r:
                cp_r = min(temp_r, key=lambda p: p[1])
                pts_r.append(cp_r[:])

            # 退出条件: 连续3次不动 或 左右收敛
            if len(pts_l) > 3 and len(pts_r) > 2:
                if (pts_r[-1] == pts_r[-2] == pts_r[-3]) or \
                   (pts_l[-1] == pts_l[-2] == pts_l[-3]):
                    break
                if (abs(pts_r[-1][0] - pts_l[-1][0]) < 2 and
                        abs(pts_r[-1][1] - pts_l[-1][1]) < 2):
                    break

        return pts_l, pts_r

    # ================================================================
    # 最小二乘直线拟合
    # ================================================================

    @staticmethod
    def _FitMidLine(mid_points):
        """最小二乘拟合 x = k·y + b, y 归一化避免矩阵病态"""
        n = len(mid_points)
        if n < 3:
            return 0.0, 0.0

        y_max = float(max(p[1] for p in mid_points))
        if y_max < 1.0:
            y_max = 1.0

        sum_y = sum_y2 = 0.0
        sum_x = sum_xy = 0.0

        for x, y_coord in mid_points:
            y_f = float(y_coord) / y_max          # 归一化到 [0, 1]
            sum_y  += y_f
            sum_y2 += y_f * y_f
            sum_x  += x
            sum_xy += x * y_f

        det = n * sum_y2 - sum_y * sum_y
        if abs(det) < 1e-12:
            return 0.0, 0.0

        inv_det = 1.0 / det
        b_n = (sum_x * sum_y2 - sum_y * sum_xy) * inv_det
        k_n = (n * sum_xy - sum_y * sum_x) * inv_det

        # 反归一化: x = k_n*(y/y_max) + b_n = (k_n/y_max)*y + b_n
        k = k_n / y_max
        b = b_n

        return k, b

    # ================================================================
    # 丢线处理
    # ================================================================

    def _HandleInvalid(self):
        self._consecutive_invalid += 1
        self.last_left_pts  = []
        self.last_right_pts = []
        self.last_mid_pts   = []
        self.last_k = 0.0
        self.last_b = 0.0
        self._result = {"offset": 0.0, "angle": 0.0, "valid": False}
