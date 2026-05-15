"""
可视化调试模块

在图像上绘制检测结果的叠加层，方便测试时观察算法中间结果。
所有绘制函数直接修改传入的 image 对象。

用法:
  from visualizer import draw_lane_hud

  # 在原始图上叠加 HUD (文字 + ROI线 + 中心十字)
  draw_lane_hud(img, fps=50, mode="LANE", offset=-3.2, angle=1.5, valid=True)
"""

import config

# ============================================================
# 颜色常量 (RGB565 三元组)
# ============================================================

COLOR_GREEN   = (0, 255, 0)
COLOR_RED     = (255, 0, 0)
COLOR_BLUE    = (0, 0, 255)
COLOR_YELLOW  = (255, 255, 0)
COLOR_CYAN    = (0, 255, 255)
COLOR_WHITE   = (255, 255, 255)
COLOR_ORANGE  = (255, 165, 0)
COLOR_BLACK   = (0, 0, 0)


# ============================================================
# 主图叠加 — HUD 信息
# ============================================================


def draw_lane_hud(img, fps=0.0, mode="", offset=0.0, angle=0.0, valid=False):
    """
    在原始图上绘制车道线检测的 HUD 信息

    绘制内容:
      - 黄色横线标记 ROI 起始位置
      - 左上角 FPS / Mode / Offset / Angle 文字
      - 图像中心十字线
      - 丢线时显示红色 INVALID 警告

    参数:
        img    : 原始 RGB565 图像 (sensor.snapshot 返回的对象)
        fps    : 当前帧率
        mode   : 当前工作模式名 (如 "LANE")
        offset : 车道偏移 (mm), 正=偏右, 负=偏左
        angle  : 车身朝向角 (°)
        valid  : 本帧检测是否有效
    """
    w = config.IMAGE_WIDTH
    h = config.IMAGE_HEIGHT

    # -- ROI 起始线 --
    y_roi = int(h * config.ROI_Y_START_RATIO)
    img.draw_line(0, y_roi, w, y_roi, color=COLOR_YELLOW, thickness=1)

    # -- HUD 文字 (左上角, 行间距 10px) --
    y = 2
    img.draw_string(2, y, "FPS:%.1f" % fps, color=COLOR_GREEN, scale=1)
    y += 10
    img.draw_string(2, y, "Mode:%s" % mode, color=COLOR_CYAN, scale=1)
    y += 10

    if valid:
        img.draw_string(2, y, "Off:%+.1fmm" % offset, color=COLOR_GREEN, scale=1)
        y += 10
        img.draw_string(2, y, "Ang:%+.1fdeg" % angle,
                        color=COLOR_GREEN, scale=1)
    else:
        img.draw_string(2, y, "INVALID", color=COLOR_RED, scale=1)

    # -- 图像中心十字 --
    cx = w // 2
    cy = h // 2
    img.draw_cross(cx, cy, color=COLOR_WHITE, size=8, thickness=1)


# ============================================================
# 鸟瞰图叠加 — 车道线点云 + 拟合曲线 (暂不使用, 需要时解除注释)
# ============================================================

# def draw_lane_birdview(lane_detector):
#     """
#     在车道检测的鸟瞰图上叠加点云和拟合曲线
#
#     对 warped 灰度图 (Canny 边缘) 做副本，然后绘制:
#       - 左车道线点云 (中灰 128, 降低视觉干扰)
#       - 右车道线点云 (亮灰 192)
#       - 二次拟合曲线 (白 255)
#       - 车道中心线 (白 255)
#
#     参数:
#         lane_detector: LaneDetector 实例
#
#     返回:
#         Image: 带叠加层的灰度鸟瞰图副本
#         None:  无数据 (last_warped 为 None)
#     """
#     warped = lane_detector.last_warped
#     if warped is None:
#         return None
#
#     viz = warped.copy()
#
#     # -- 左车道线点云 (中灰小圆点) --
#     for x, y in lane_detector.last_left_pts:
#         if 0 <= x < viz.width() and 0 <= y < viz.height():
#             viz.draw_circle(x, y, 1, color=128, fill=True)
#
#     # -- 右车道线点云 (亮灰小圆点) --
#     for x, y in lane_detector.last_right_pts:
#         if 0 <= x < viz.width() and 0 <= y < viz.height():
#             viz.draw_circle(x, y, 1, color=192, fill=True)
#
#     # -- 拟合曲线 --
#     left_coeff  = lane_detector.last_left_coeff
#     right_coeff = lane_detector.last_right_coeff
#
#     if left_coeff is not None:
#         _DrawQuadratic(viz, left_coeff, val=255)
#     if right_coeff is not None:
#         _DrawQuadratic(viz, right_coeff, val=255)
#
#     # -- 车道中心线 --
#     if left_coeff is not None and right_coeff is not None:
#         a_l, b_l, c_l = left_coeff
#         a_r, b_r, c_r = right_coeff
#         prev_cx = prev_cy = None
#         for y in range(0, viz.height(), 3):
#             x_l = a_l * y * y + b_l * y + c_l
#             x_r = a_r * y * y + b_r * y + c_r
#             cx = int((x_l + x_r) / 2)
#             if 0 <= cx < viz.width():
#                 if prev_cx is not None:
#                     viz.draw_line(prev_cx, prev_cy, cx, y, color=255, thickness=1)
#                 prev_cx, prev_cy = cx, y
#
#     # -- 鸟瞰图水平中线 --
#     mid_y = viz.height() // 2
#     viz.draw_line(0, mid_y, viz.width(), mid_y, color=128, thickness=1)
#
#     return viz
#
#
# # ============================================================
# # 辅助函数 (鸟瞰图用, 暂不使用)
# # ============================================================
#
#
# def _DrawQuadratic(img, coeff, val=255):
#     """
#     在灰度图上用线段连接方式绘制 x = a·y² + b·y + c 曲线
#
#     参数:
#         img   : 灰度图
#         coeff : (a, b, c) 拟合系数
#         val   : 绘制灰度值
#     """
#     a, b, c = coeff
#     w = img.width()
#     h = img.height()
#     prev_x = None
#     prev_y = None
#
#     for y in range(0, h, 3):
#         x = int(a * y * y + b * y + c)
#         if 0 <= x < w:
#             if prev_x is not None:
#                 img.draw_line(prev_x, prev_y, x, y, color=val, thickness=1)
#             prev_x, prev_y = x, y
