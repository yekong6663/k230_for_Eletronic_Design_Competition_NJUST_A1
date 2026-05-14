"""
与 M0 的 UART 通讯协议

帧格式 (共 12 字节):
  [0xAA] [帧类型] [数据段 8B] [XOR校验]

车道线帧 (0x01):
  | 0xAA | 0x01 | offset(4B float LE) | angle(4B float LE) | valid(1B) | XOR |
  - offset : 横向偏移 (mm)，左正右负
  - angle  : 航向偏差 (°)，左偏为正
  - valid  : 1=有效, 0=丢线
  - XOR    : 前 11 字节异或
"""

import struct
from machine import UART
import config


class UartProtocol:
    """M0 ↔ K230 通讯协议封装"""

    def __init__(self):
        self._uart = UART(config.UART_ID, baudrate=config.UART_BAUD)

    # ============================================================
    # 发送接口
    # ============================================================

    def SendLane(self, offset, angle, valid):
        """
        发送车道线数据帧 (0x01, 12 字节)
        offset : float, 横向偏移 (mm), 左正右负
        angle  : float, 航向偏差 (°), 左偏为正
        valid  : bool,  True=有效, False=丢线
        """
        data = struct.pack("<ffB", offset, angle, 1 if valid else 0)
        self._SendFrame(0x01, data)

    # ============================================================
    # 接收接口
    # ============================================================

    def ReadCommand(self):
        """
        非阻塞读取 M0 发来的一个字节指令
        返回: int (0x10~0x1F) 或 None (无数据)
        """
        if self._uart.any():
            return self._uart.read(1)[0]
        return None

    # ============================================================
    # 内部
    # ============================================================

    def _SendFrame(self, frame_type, payload):
        """打包并发送完整帧"""
        frame = bytes([config.FRAME_HEADER, frame_type]) + payload
        checksum = self._XorChecksum(frame)
        self._uart.write(frame + bytes([checksum]))

    @staticmethod
    def _XorChecksum(data):
        """逐字节异或校验"""
        result = 0
        for b in data:
            result ^= b
        return result
