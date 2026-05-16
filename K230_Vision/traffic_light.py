"""
红绿灯识别模块

算法管线:
  RGB → HSV → 颜色分割(红/绿) → 形态学开运算 → 面积滤波 → 多帧确认
  - 红色跨 0°: HSV(0~10) ∪ HSV(170~180)
  - 绿色: HSV(35~85)
  - 以后加入标识，主要是转弯方向和相关禁行/限速

帧格式 (0x03):
  | 0xAA | 0x03 | light_status(1B) | reserved(1B) | XOR |
  - status: 0=无灯, 1=红灯, 2=绿灯
"""

from detector_base import Detector


class TrafficLightDetector(Detector):
    """红绿灯检测器，继承自 Detector 基类"""

    def __init__(self):
        super().__init__(name="TrafficLight")

        # 结果: 0=无灯, 1=红灯, 2=绿灯
        self._result = {"status": 0}

        # 多帧确认计数器
        self._frames_red   = 0   # 连续检测到红色的帧数
        self._frames_green = 0   # 连续检测到绿色的帧数

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        """初始化红绿灯检测器"""
        super().Init()
        self._result = {"status": 0}
        self._frames_red   = 0
        self._frames_green = 0

    def Process(self, img):
        """
        检测红绿灯状态

        参数:
            img: OpenMV image 对象 (RGB565)

        返回:
            int or None: 1=红灯, 2=绿灯, None=无变化不需发帧
        """
        # TODO: HSV 分割 + 形态学 + 面积滤波 + 多帧确认
        return None

    def Reset(self):
        """重置多帧确认计数器"""
        self._result = {"status": 0}
        self._frames_red   = 0
        self._frames_green = 0

    def SendToUart(self):
        """发送红绿灯检测结果 (待实现)"""
        pass
