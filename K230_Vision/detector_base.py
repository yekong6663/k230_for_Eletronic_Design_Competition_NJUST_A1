"""
检测器基类

所有视觉检测模块 (车道线、红绿灯、路口、停车位、道闸)
均继承此基类，统一对外接口:

  - Init()       : 初始化本检测器的内部状态
  - Process(img) : 对一帧图像执行检测，返回结果 dict 或 None
  - GetResult()  : 获取最近一次检测结果 (只读，不重新计算)
  - Reset()      : 重置检测器内部状态 (丢线计数清零等)

子类可额外公开属性供其他模块读取 (如 lane.last_left_coeff)
"""


class Detector:
    """视觉检测器基类，所有检测器必须继承并重写 Process()"""

    def __init__(self, name="Detector"):
        # 检测器名称，用于日志打印
        self._name = name
        # 最近一次检测结果，子类在 Process() 中更新
        self._result = {}
        # 初始化标志
        self._initialized = False

    def Init(self):
        """
        初始化检测器内部状态

        基类默认只标记已初始化。
        子类可重写此方法，执行额外的初始化逻辑
        (如清空计数器、加载标定参数等)。
        """
        self._initialized = True

    def Process(self, img):
        """
        对一帧图像执行检测

        参数:
            img: OpenMV image 对象 (RGB565 或灰度图)

        返回:
            dict 或 None:
            - dict  : 检测结果 (内容由子类定义)
            - None  : 本帧无有效结果或无需发帧

        子类必须重写此方法。
        """
        raise NotImplementedError(
            "Detector subclass must override Process()"
        )

    def GetResult(self):
        """
        获取最近一次检测结果 (只读)

        返回:
            dict: 最后一次 Process() 存入的结果
        """
        return self._result

    def Reset(self):
        """
        重置检测器内部状态

        子类可重写此方法，清除统计计数器、拟合系数缓存等。
        基类默认不做任何事。
        """
        pass

    @property
    def name(self):
        """检测器名称 (只读)"""
        return self._name

    @property
    def initialized(self):
        """是否已初始化 (只读)"""
        return self._initialized
