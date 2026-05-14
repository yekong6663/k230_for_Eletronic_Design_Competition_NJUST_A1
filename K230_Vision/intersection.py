"""
路口检测模块

算法管线:
  车道线间距突变检测 + 鸟瞰图上方 Sobel 横向边缘投影
  → 类型判定(十字/T字左/T字右) → 多帧确认

依赖:
  - lane_detect.last_left_coeff / last_right_coeff
  - lane_detect.last_warped

帧格式 (0x02):
  | 0xAA | 0x02 | intersection_type(1B) | direction(1B) | XOR |
  - type: 0=无, 1=十字, 2=T字左, 3=T字右
  - direction: 0=无, 1=左转, 2=右转, 3=直行
"""

from detector_base import Detector


class IntersectionDetector(Detector):
    """路口检测器，继承自 Detector 基类"""

    def __init__(self):
        super().__init__(name="Intersection")

        # 结果: type=0~3, direction=0~3
        self._result = {"type": 0, "direction": 0}

        # 多帧确认
        self._confirm_count = 0   # 连续检测到同类型路口的帧数
        self._last_type     = 0   # 上一帧检测到的路口类型

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        """初始化路口检测器"""
        super().Init()
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0

    def Process(self, warped, left_coeff, right_coeff):
        """
        检测路口类型

        参数:
            warped      : 鸟瞰图 (来自 lane_detect.last_warped)
            left_coeff  : 左车道线拟合系数 (a,b,c) or None
            right_coeff : 右车道线拟合系数 (a,b,c) or None

        返回:
            (type, direction) or None (None=无路口)
        """
        # TODO: 间距突变 + Sobel 投影 + 类型判定 + 多帧确认
        return None

    def Reset(self):
        """重置路口检测器状态"""
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0
