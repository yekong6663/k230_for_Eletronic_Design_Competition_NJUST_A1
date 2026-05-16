# ============================================================
# 全局配置 & 阈值参数
# 所有可调参数集中于此，方便联调时快速修改
# ============================================================

# --- 图像参数 ---
IMAGE_WIDTH  = 400
IMAGE_HEIGHT = 240
FPS          = 90

# --- Sensor 参数 ---
SENSOR_ID = 2               # 摄像头 ID

# --- UART 参数 ---
UART_ID   = 2             # UART2 (对应 GPIO11=TX, GPIO12=RX)
UART_BAUD = 115200
UART_TX   = 11            # UART2 TXD 引脚号
UART_RX   = 12            # UART2 RXD 引脚号

# --- 帧头 ---
FRAME_HEADER = 0xAA

# --- ROI 参数 ---
ROI_Y_START_RATIO = 0.40   # ROI 起始行比例 (丢弃天空等无关区域)
ROI_Y_END_RATIO   = 0.95   # ROI 结束行比例 (丢弃车头区域)

# --- 车道线拟合参数 ---
FIT_MIN_POINTS = 8          # 单侧拟合最少点数，低于此值认为丢线

# --- 鸟瞰图尺度 ---
MM_PER_PIXEL = 1.0           # 直接使用像素偏移, 不做 mm 转换

# --- 丢线判定 ---
MAX_CONSECUTIVE_INVALID = 5 # 连续丢线帧数上报 (超过后 valid=0)

# --- 可视化调试 ---
ENABLE_VIZ = True            # 是否在原始图上叠加 HUD 信息 (ROI线 + FPS + offset/angle)

# --- 显示参数 ---
DISPLAY_MODE = "VIRT"        # "VIRT"=IDE虚拟显示, "LCD"=3.1寸屏幕(800x480), "HDMI"=HDMI扩展板
LCD_WIDTH    = 800           # ST7701 LCD 物理分辨率宽
LCD_HEIGHT   = 480           # ST7701 LCD 物理分辨率高
