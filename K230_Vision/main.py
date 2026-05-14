"""
K230 视觉系统 —— 主入口

功能:
  - 初始化摄像头、UART、所有检测器
  - 主循环: 取图 → 车道线检测 → 发送结果 → 接收 M0 指令切换模式
  - 当前已实现: 车道线巡线
  - 后续待叠加: 红绿灯 / 路口 / 停车位 / 道闸

框架:
  - 所有检测器继承 Detector 基类，统一 Init() / Process() / GetResult() 接口
  - 主循环使用 while True + try-except 保证异常可恢复
"""

import sensor
import image
import time
import config
from uart_protocol import UartProtocol
from lane_detect import LaneDetector
from traffic_light import TrafficLightDetector
from intersection import IntersectionDetector
from parking_spot import ParkingSpotDetector
from gate_detect import GateDetector
from visualizer import draw_lane_hud

# ============================================================
# 模式常量 — M0 发来的指令码映射到工作模式
# ============================================================

# M0 → K230 指令码 (1 字节)
CMD_LANE  = 0x10    # 巡线模式
CMD_INTER = 0x11    # 路口检测模式
CMD_PARK  = 0x12    # 停车位检测模式
CMD_LIGHT = 0x13    # 红绿灯检测模式
CMD_GATE  = 0x14    # 道闸识别模式
CMD_IDLE  = 0x15    # 空闲模式

# 指令码 → 模式名 映射表
CMD_TO_MODE = {
    CMD_LANE:  "LANE",
    CMD_INTER: "INTER",
    CMD_PARK:  "PARK",
    CMD_LIGHT: "LIGHT",
    CMD_GATE:  "GATE",
    CMD_IDLE:  "IDLE",
}


# ============================================================
# 系统初始化
# ============================================================


def _InitSensor():
    """
    初始化摄像头

    配置: RGB565 色彩, QVGA (320×240), 50fps
    开启自动白平衡和自动曝光，适应场地光照变化。
    """
    sensor.reset()                          # 复位摄像头
    sensor.set_pixformat(sensor.RGB565)     # 色彩格式: RGB565
    sensor.set_framesize(sensor.QVGA)       # 分辨率: 320×240
    sensor.set_auto_whitebal(True)          # 自动白平衡 (色温自适应)
    sensor.set_auto_exposure(True)          # 自动曝光 (亮度自适应)
    sensor.set_framerate(config.FPS)        # 帧率: 50fps
    sensor.skip_frames(20)                  # 跳过启动初期的 20 帧 (曝光不稳定)


def _InitAll():
    """
    初始化全部模块并返回实例

    执行顺序:
      1. 摄像头初始化
      2. 通讯协议实例化 (UART)
      3. 各检测器实例化 + Init()

    返回:
        (uart, lane, light, inter, park, gate) 六个模块实例
    """
    # -- 摄像头 --
    _InitSensor()

    # -- 通讯协议 (UART1, 115200bps) --
    uart = UartProtocol()

    # -- 检测器: 实例化 + 初始化 --
    lane  = LaneDetector()
    light = TrafficLightDetector()
    inter = IntersectionDetector()
    park  = ParkingSpotDetector()
    gate  = GateDetector()

    # 每个检测器调用 Init() 设置内部状态
    lane.Init()
    light.Init()
    inter.Init()
    park.Init()
    gate.Init()

    # -- 启动日志 --
    print("=" * 48)
    print("  K230 Vision System — NJUST A1")
    print("  Resolution : %dx%d @ %d fps" % (config.IMAGE_WIDTH, config.IMAGE_HEIGHT, config.FPS))
    print("  UART%d      : %d bps" % (config.UART_ID, config.UART_BAUD))
    print("  Detectors  : Lane / Light / Inter / Park / Gate")
    print("  Mode       : LANE (巡线)")
    print("=" * 48)

    return uart, lane, light, inter, park, gate


# ============================================================
# 主循环
# ============================================================


def Main():
    """
    K230 视觉系统主循环

    流程:
      while True:
        1. clock.tick()  — 开始帧计时
        2. 取一帧图像
        3. 车道线检测 (始终运行)
        4. 根据当前 mode 调用对应检测器 (待接入)
        5. 检查 M0 指令切换模式
        6. 异常捕获 → 短暂延迟后继续 (不死机)

    帧率受 sensor.set_framerate() 限制为 50fps。
    每次 snapshot() 会阻塞直到新帧就绪。
    """
    # -- 初始化所有硬件和检测器 --
    uart, lane, light, inter, park, gate = _InitAll()

    # -- 时钟对象，用于统计帧率和帧间隔 --
    clock = time.clock()

    # -- 默认模式: 巡线 --
    mode = "LANE"

    # ================================================================
    # 主循环
    # ================================================================
    while True:
        # ---- 开始帧计时 (记录本帧开始时刻) ----
        clock.tick()

        try:
            # ---- ① 取一帧图像 ----
            # snapshot() 会阻塞，直到摄像头曝光完成返回新帧
            img = sensor.snapshot()

            # ---- ② 车道线检测 (始终运行，所以所有模式都有 lane 数据) ----
            lane_result = lane.Process(img)

            if lane_result["valid"]:
                # 双线有效 → 发送正常数据
                uart.SendLane(
                    lane_result["offset"],
                    lane_result["angle"],
                    True
                )
            else:
                # 丢线 → 发送无效标记 (M0 侧根据 valid 做降速/停车)
                uart.SendLane(0.0, 0.0, False)

            # ========================================================
            # TODO: 以下为后续接入其他检测器的代码框架
            # ========================================================
            #
            # if mode == "LIGHT":
            #     # 红绿灯检测 (巡线同时扫描)
            #     light_result = light.Process(img)
            #     if light_result is not None:
            #         uart.SendLight(light_result)
            #
            # elif mode == "INTER":
            #     # 路口检测 (读取车道线拟合中间结果)
            #     inter_result = inter.Process(
            #         lane.last_warped,
            #         lane.last_left_coeff,
            #         lane.last_right_coeff
            #     )
            #     if inter_result is not None:
            #         uart.SendIntersection(*inter_result)
            #
            # elif mode == "PARK":
            #     # 停车位检测
            #     park_result = park.Process(img)
            #     if park_result is not None:
            #         uart.SendParkingSpot(*park_result)
            #
            # elif mode == "GATE":
            #     # 道闸识别
            #     gate_result = gate.Process(img)
            #     if gate_result is not None:
            #         uart.SendGate(gate_result)
            #
            # ========================================================

            # ---- ③ 可视化叠加层 (调试用, 生产时可关闭) ----
            if config.ENABLE_VIZ:
                draw_lane_hud(
                    img,
                    fps=clock.fps(),
                    mode=mode,
                    offset=lane_result["offset"],
                    angle=lane_result["angle"],
                    valid=lane_result["valid"],
                )

            # ---- ④ 接收 M0 指令，切换工作模式 ----
            cmd = uart.ReadCommand()
            if cmd is not None and cmd in CMD_TO_MODE:
                new_mode = CMD_TO_MODE[cmd]
                if new_mode != mode:
                    mode = new_mode
                    print("[Mode] 切换到:", mode)

        except Exception as e:
            # 异常捕获: 打印错误信息，短暂延迟后继续
            # 防止单帧处理异常导致整个系统崩溃
            print("[Error]", e)
            time.sleep_ms(10)      # 等待 10ms 让日志输出完成

        # ---- 可选: 打印帧率 (调试用) ----
        # print("FPS:", clock.fps())


# ============================================================
# 程序入口 — 上电自动执行
# ============================================================

if __name__ == "__main__":
    Main()
