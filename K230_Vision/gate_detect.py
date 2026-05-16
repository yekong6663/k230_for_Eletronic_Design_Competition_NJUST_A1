"""
道闸识别模块

算法管线:
  HSV 颜色分割(红/黄色) → 形态学闭运算 → 最小外接矩形(minAreaRect)
  → 倾角判定: <20° 落下, >70° 抬起

帧格式 (0x05):
  | 0xAA | 0x05 | gate_status(1B) | reserved(1B) | XOR |
  - status: 0=未检测到, 1=落下(不可通行), 2=抬起(可通行)
"""

from detector_base import Detector


class GateDetector(Detector):
    """道闸检测器，继承自 Detector 基类"""

    def __init__(self):
        super().__init__(name="GateDetector")

        # 结果: 0=未检测到, 1=落下, 2=抬起
        self._result = {"status": 0}

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        """初始化道闸检测器"""
        super().Init()
        self._result = {"status": 0}

    def Process(self, img):
        """
        检测道闸状态

        参数:
            img: OpenMV image 对象 (RGB565)

        返回:
            int or None: 1=落下, 2=抬起, None=无变化
        """
        # TODO: 颜色分割 → 最小外接矩形 → 倾角判定
        return None

    def Reset(self):
        """重置道闸检测器状态"""
        self._result = {"status": 0}

    def SendToUart(self):
        """发送道闸检测结果 (待实现)"""
        pass
