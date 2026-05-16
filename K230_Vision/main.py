"""
K230 视觉系统 —— 主入口

功能:
  - 初始化摄像头、UART、所有检测器
  - 主循环: 取图 → 车道线检测 → 显示 → 接收指令
  - 路口检测在独立线程运行 (小核)

框架:
  - 所有检测器继承 Detector 基类
  - 每个检测器通过 SendToUart() 自行发送结果
"""

import _thread
import time
import os
import image
from media.sensor import *
from media.display import *
from media.media import *
import config
from uart_protocol import UartProtocol
from lane_detect import LaneDetector
from intersection import IntersectionDetector
from parking_spot import ParkingSpotDetector
from gate_detect import GateDetector
from visualizer import draw_lane_hud, draw_lane_overlay, draw_intersection_boxes

# ============================================================
# 模式常量
# ============================================================

CMD_LANE  = 0x10
CMD_INTER = 0x11
CMD_PARK  = 0x12
CMD_LIGHT = 0x13
CMD_GATE  = 0x14
CMD_IDLE  = 0x15

CMD_TO_MODE = {
    CMD_LANE:  "LANE",
    CMD_INTER: "INTER",
    CMD_PARK:  "PARK",
    CMD_LIGHT: "LIGHT",
    CMD_GATE:  "GATE",
    CMD_IDLE:  "IDLE",
}

# ============================================================
# 路口检测线程 — 共享数据
# ============================================================

_inter_img   = None      # 主线程写入的二值化图像副本
_inter_ready = False     # True = 新帧就绪


def _InterThread(inter):
    """路口检测线程 (运行在小核上)"""
    global _inter_img, _inter_ready

    while True:
        if _inter_ready:
            img = _inter_img
            _inter_ready = False
            inter.Process(img, [], [])
            inter.SendToUart()
        time.sleep_ms(5)


# ============================================================
# 系统初始化
# ============================================================


def _InitSensor():
    """初始化摄像头: GRAYSCALE, 400×240, 90fps"""
    sensor = Sensor(
        id=config.SENSOR_ID,
        width=config.IMAGE_WIDTH,
        height=config.IMAGE_HEIGHT,
        fps=config.FPS,
    )
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(width=config.IMAGE_WIDTH, height=config.IMAGE_HEIGHT,
                         chn=CAM_CHN_ID_0)
    sensor.set_pixformat(Sensor.GRAYSCALE, chn=CAM_CHN_ID_0)
    sensor.run()
    time.sleep_ms(500)
    return sensor


def _InitAll():
    """初始化全部模块"""
    MediaManager.init()

    sensor = _InitSensor()
    uart   = UartProtocol()

    lane  = LaneDetector()
    inter = IntersectionDetector()
    park  = ParkingSpotDetector()
    gate  = GateDetector()

    # 注入 UART
    lane.SetUart(uart)
    inter.SetUart(uart)
    park.SetUart(uart)
    gate.SetUart(uart)

    # 初始化
    lane.Init()
    inter.Init()
    park.Init()
    gate.Init()

    # -- 启动路口检测线程 --
    _thread.start_new_thread(_InterThread, (inter,))

    # -- 日志 --
    print("=" * 48)
    print("  K230 Vision System — NJUST A1")
    print("  Resolution : %dx%d @ %d fps" % (
        config.IMAGE_WIDTH, config.IMAGE_HEIGHT, config.FPS))
    print("  UART%d      : %d bps" % (config.UART_ID, config.UART_BAUD))
    print("  Detectors  : Lane / Inter / Park / Gate")
    print("  Threads    : Main + Inter(LittleCore)")
    print("=" * 48)

    return sensor, uart, lane, inter, park, gate


# ============================================================
# 主循环
# ============================================================


def Main():
    sensor, uart, lane, inter, park, gate = _InitAll()

    # -- 显示初始化 --
    if config.DISPLAY_MODE == "LCD":
        disp_w, disp_h = config.LCD_WIDTH, config.LCD_HEIGHT
        Display.init(Display.ST7701, width=disp_w, height=disp_h, to_ide=True)
        disp_img = image.Image(disp_w, disp_h, image.ARGB8888)
    elif config.DISPLAY_MODE == "HDMI":
        disp_w, disp_h = 1920, 1080
        Display.init(Display.LT9611, width=disp_w, height=disp_h, to_ide=True)
        disp_img = image.Image(disp_w, disp_h, image.ARGB8888)
    else:
        disp_w, disp_h = config.IMAGE_WIDTH, config.IMAGE_HEIGHT
        Display.init(Display.VIRT, width=disp_w, height=disp_h, to_ide=True)
        disp_img = None

    clock = time.clock()
    mode  = "LANE"

    global _inter_img, _inter_ready

    # ================================================================
    # 主循环
    # ================================================================
    try:
        while True:
            clock.tick()

            try:
                # ---- ① 取帧 + 车道线检测 (原地二值化) ----
                img = sensor.snapshot(chn=CAM_CHN_ID_0)
                lane_result = lane.Process(img)

                # ---- ② 发送车道线结果 ----
                lane.SendToUart()

                # ---- ③ 提交二值化帧给路口检测线程 ----
                if not _inter_ready:
                    _inter_img = img.copy()
                    _inter_ready = True

                # ---- ④ 可视化叠加层 ----
                if config.ENABLE_VIZ:
                    draw_lane_overlay(img, lane)
                    draw_intersection_boxes(img, inter)
                    draw_lane_hud(
                        img,
                        fps=clock.fps(),
                        mode=mode,
                        offset=lane_result["offset"],
                        angle=lane_result["angle"],
                        valid=lane_result["valid"],
                        intersection=inter._result["type"],
                    )

                # ---- ⑤ 显示 ----
                if disp_img is not None:
                    disp_img.clear()
                    disp_img.draw_image(img, 0, 0,
                                        x_scale=disp_w / img.width(),
                                        y_scale=disp_h / img.height())
                    Display.show_image(disp_img)
                else:
                    Display.show_image(img)

                # ---- ⑥ 接收 M0 指令 ----
                cmd = uart.ReadCommand()
                if cmd is not None and cmd in CMD_TO_MODE:
                    new_mode = CMD_TO_MODE[cmd]
                    if new_mode != mode:
                        mode = new_mode
                        print("[Mode] 切换到:", mode)

            except Exception as e:
                print("[Error]", e)
                time.sleep_ms(10)

            os.exitpoint()

    except KeyboardInterrupt:
        print("用户终止")
    finally:
        Display.deinit()
        os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
        time.sleep_ms(100)
        MediaManager.deinit()


# ============================================================
# 程序入口
# ============================================================

if __name__ == "__main__":
    Main()
