"""
检测器基类

所有视觉检测模块 (车道线、红绿灯、路口、停车位、道闸)
均继承此基类，统一对外接口:

  - Init()        : 初始化本检测器的内部状态
  - Process(img)  : 对一帧图像执行检测，返回结果 dict 或 None
  - GetResult()   : 获取最近一次检测结果 (只读，不重新计算)
  - Reset()       : 重置检测器内部状态 (丢线计数清零等)
  - SendToUart()  : 将最新检测结果通过 UART 发送 (子类必须重写)
"""


class Detector:
    """视觉检测器基类，所有检测器必须继承并重写 Process() 和 SendToUart()"""

    def __init__(self, name="Detector"):
        self._name = name
        self._result = {}
        self._initialized = False
        self._uart = None         # UartProtocol 实例, 由主循环注入

    def SetUart(self, uart):
        """注入 UART 通讯实例，供 SendToUart 使用"""
        self._uart = uart

    def Init(self):
        """初始化检测器内部状态"""
        self._initialized = True

    def Process(self, img):
        """
        对一帧图像执行检测

        参数:
            img: 图像对象

        返回:
            dict 或 None:
            - dict  : 检测结果 (内容由子类定义)
            - None  : 本帧无有效结果或无需发帧

        子类必须重写此方法。
        """
        raise NotImplementedError(
            "Detector subclass must override Process()"
        )

    def SendToUart(self):
        """
        将最近一次检测结果通过 UART 发送

        子类必须重写此方法，调用 self._uart 的对应发送函数。
        """
        raise NotImplementedError(
            "Detector subclass must override SendToUart()"
        )

    def GetResult(self):
        """获取最近一次检测结果 (只读)"""
        return self._result

    def Reset(self):
        """重置检测器内部状态"""
        pass

    @property
    def name(self):
        return self._name

    @property
    def initialized(self):
        return self._initialized
