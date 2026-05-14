"""
停车位检测模块

算法管线:
  自适应二值化 → 矩形检测 → 几何筛选(面积/长宽比/凸度)
  → 框内方差分析(空/有车) → 逆透视变换 → 世界坐标(mm)

帧格式 (0x04):
  | 0xAA | 0x04 | spot_x(2B int16 LE) | spot_y(2B int16 LE) |
            spot_w(2B int16 LE) | spot_angle(2B int16 LE) | XOR |
  - spot_x/y   : 空车位中心相对车头坐标 (mm)
  - spot_w     : 车位宽度 (mm)
  - spot_angle : 车位方向角 (°), 平行车道=0, 垂直=90
"""

from detector_base import Detector


class ParkingSpotDetector(Detector):
    """停车位检测器，继承自 Detector 基类"""

    def __init__(self):
        super().__init__(name="ParkingSpot")

        # 结果: x,y,w,angle 为空车位信息, found 标记是否找到
        self._result = {
            "x": 0, "y": 0, "w": 0, "angle": 0,
            "found": False
        }

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        """初始化停车位检测器"""
        super().Init()
        self._result = {
            "x": 0, "y": 0, "w": 0, "angle": 0,
            "found": False
        }

    def Process(self, img):
        """
        检测空车位

        参数:
            img: OpenMV image 对象 (RGB565)

        返回:
            (x, y, w, angle) or None (None=未找到空位)
        """
        # TODO: 二值化 → 矩形筛选 → 空位判定 → 坐标转换
        return None

    def Reset(self):
        """重置停车位检测器状态"""
        self._result = {
            "x": 0, "y": 0, "w": 0, "angle": 0,
            "found": False
        }
