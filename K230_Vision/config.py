# ============================================================
# 全局配置 & 阈值参数
# 所有可调参数集中于此，方便联调时快速修改
# ============================================================

# --- 图像参数 ---
IMAGE_WIDTH  = 320
IMAGE_HEIGHT = 240
FPS          = 50

# --- UART 参数 ---
UART_ID   = 1
UART_BAUD = 115200

# --- 帧头 ---
FRAME_HEADER = 0xAA

# --- ROI 参数 ---
ROI_Y_START_RATIO = 0.40   # ROI 起始行比例 (丢弃天空等无关区域)
ROI_Y_END_RATIO   = 0.95   # ROI 结束行比例 (丢弃车头区域)

# --- 预处理参数 ---
GAUSSIAN_KERNEL = 3         # 高斯滤波核大小 (奇数)
CANNY_LOW       = 40        # Canny 低阈值
CANNY_HIGH      = 100       # Canny 高阈值

# --- 车道线拟合参数 ---
FIT_MIN_POINTS = 8          # 单侧拟合最少点数，低于此值认为丢线

# --- 鸟瞰图尺度 ---
MM_PER_PIXEL = 1.25         # 鸟瞰图像素到 mm 的比例 (需标定确认)

# --- 丢线判定 ---
MAX_CONSECUTIVE_INVALID = 5 # 连续丢线帧数上报 (超过后 valid=0)

# --- 可视化调试 ---
ENABLE_VIZ = True            # 是否在原始图上叠加 HUD 信息 (ROI线 + FPS + offset/angle)
