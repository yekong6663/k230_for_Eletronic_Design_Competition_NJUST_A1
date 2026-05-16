"""
可视化调试模块

在灰度图像上绘制检测结果的叠加层，方便测试时观察算法中间结果。
所有绘制函数直接修改传入的 image 对象。

用法:
  from visualizer import draw_lane_hud, draw_lane_overlay

  draw_lane_hud(img, fps=90, mode="LANE", offset=-3.2, angle=1.5, valid=True)
  draw_lane_overlay(img, lane_detector)
"""

import config

# ============================================================
# 灰度常量 (0=黑, 255=白)
# ============================================================

GRAY_WHITE   = 255
GRAY_LIGHT   = 200
GRAY_MEDIUM  = 128
GRAY_DARK    = 60
GRAY_BLACK   = 0


# ============================================================
# 主图叠加 — HUD 信息
# ============================================================


def draw_lane_hud(img, fps=0.0, mode="", offset=0.0, angle=0.0,
                  valid=False, intersection=0):
    """
    在灰度图上绘制车道线检测的 HUD 信息

    绘制内容:
      - 左上角 FPS / Mode / Offset / Angle 文字
      - 图像中心十字线
      - 丢线时显示 INVALID 警告
      - 路口检测时显示类型文字

    参数:
        img          : 灰度图 (GRAYSCALE)
        fps          : 当前帧率
        mode         : 当前工作模式名 (如 "LANE")
        offset       : 车道偏移 (mm), 正=偏右, 负=偏左
        angle        : 车身朝向角 (°)
        valid        : 本帧检测是否有效
        intersection : 路口类型 0=无, 1=十字, 2=T字左, 3=T字右
    """
    INTER_NAMES = {0: "", 1: "CROSS", 2: "T-LEFT", 3: "T-RIGHT"}

    w = config.IMAGE_WIDTH
    h = config.IMAGE_HEIGHT

    # -- HUD 文字 (左上角, 字号 12, 行间距 14px) --
    FONT_SIZE = 12
    y = 2
    img.draw_string_advanced(2, y, FONT_SIZE,
                              "FPS:%.1f" % fps, color=GRAY_LIGHT)
    y += 14
    img.draw_string_advanced(2, y, FONT_SIZE,
                              "Mode:%s" % mode, color=GRAY_MEDIUM)
    y += 14

    if valid:
        img.draw_string_advanced(2, y, FONT_SIZE,
                                  "Off:%+.1fpx" % offset, color=GRAY_LIGHT)
        y += 14
        img.draw_string_advanced(2, y, FONT_SIZE,
                                  "Ang:%+.1fdeg" % angle, color=GRAY_LIGHT)
        y += 14
    else:
        img.draw_string_advanced(2, y, FONT_SIZE,
                                  "INVALID", color=GRAY_WHITE)
        y += 14

    # -- 路口提示 --
    if intersection > 0:
        img.draw_string_advanced(2, y, FONT_SIZE,
                                 "INT:%s" % INTER_NAMES.get(intersection, "?"),
                                 color=GRAY_WHITE)

    # -- 图像中心十字 --
    cx = w // 2
    cy = h // 2
    img.draw_cross(cx, cy, color=GRAY_WHITE, size=8, thickness=1)


# ============================================================
# 车道线叠加 — 中线点 + 拟合线
# ============================================================


def draw_lane_overlay(img, lane_detector):
    """
    在灰度图上叠加车道检测的中间结果

    绘制内容:
      - 左/右车道线追踪点 (暗灰点, 不干扰二值化结果)
      - 中线点 (中灰点)
      - 拟合后的中线 (白线)

    参数:
        img            : 已二值化的灰度图
        lane_detector  : LaneDetector 实例
    """
    # -- 左车道线追踪点 --
    for x, y in lane_detector.last_left_pts:
        if 0 <= x < img.width() and 0 <= y < img.height():
            img.set_pixel(x, y, GRAY_DARK)

    # -- 右车道线追踪点 --
    for x, y in lane_detector.last_right_pts:
        if 0 <= x < img.width() and 0 <= y < img.height():
            img.set_pixel(x, y, GRAY_DARK)

    # -- 中线点 --
    for x, y in lane_detector.last_mid_pts:
        if 0 <= x < img.width() and 0 <= y < img.height():
            img.set_pixel(x, y, GRAY_MEDIUM)

    # -- 拟合中线: x = k·y + b --
    k = lane_detector.last_k
    b = lane_detector.last_b
    if k != 0.0 or b != 0.0:
        h = img.height()
        w = img.width()
        prev_xi = None
        for y in range(0, h, 3):
            xi = int(k * y + b)
            if 0 <= xi < w:
                if prev_xi is not None:
                    img.draw_line(prev_xi, y - 3, xi, y,
                                  color=GRAY_WHITE, thickness=1)
                prev_xi = xi

    # -- 底部中线预测点 (大十字标记) --
    if k != 0.0 or b != 0.0:
        y_bottom = img.height() - 1
        x_bottom = int(k * y_bottom + b)
        if 0 <= x_bottom < img.width():
            img.draw_cross(x_bottom, y_bottom, color=GRAY_WHITE,
                           size=6, thickness=1)


# ============================================================
# 路口检测框叠加
# ============================================================


def draw_intersection_boxes(img, inter):
    """
    在灰度图上绘制路口检测框和结果

    绘制内容:
      - 中间区域一排检测框 (1=白色边, 0=灰色边)
      - 框内标注 1/0
      - 框下方标注检测结果 (CROSS / T-LEFT / T-RIGHT / -)

    参数:
        img  : 已二值化的灰度图
        inter: IntersectionDetector 实例
    """
    from intersection import IntersectionDetector

    w = img.width()
    h = img.height()
    num  = IntersectionDetector.NUM_BOXES
    x0_r = IntersectionDetector.BOX_LEFT_MARGIN
    x1_r = w - IntersectionDetector.BOX_RIGHT_MARGIN
    region_w = x1_r - x0_r
    box_w = region_w // num
    box_y = IntersectionDetector.BOX_Y_OFFSET
    box_h = IntersectionDetector.BOX_HEIGHT

    # -- 画左边距分隔线 --
    img.draw_line(x0_r - 1, box_y, x0_r - 1, box_y + box_h,
                  color=GRAY_DARK, thickness=1)
    img.draw_line(x1_r, box_y, x1_r, box_y + box_h,
                  color=GRAY_DARK, thickness=1)

    # -- 画框 + 标 0/1 --
    FONT_SIZE = 10
    for i in range(num):
        x0 = x0_r + i * box_w
        x1 = min(x0 + box_w, x1_r) - 1
        y1 = min(box_y + box_h, h) - 1

        color = GRAY_WHITE if inter.box_active[i] else GRAY_MEDIUM
        img.draw_rectangle(x0, box_y, x1 - x0, y1 - box_y,
                           color=color, thickness=1)

        label = "1" if inter.box_active[i] else "0"
        label_color = GRAY_WHITE if inter.box_active[i] else GRAY_DARK
        img.draw_string_advanced(x0 + 2, box_y + 1, FONT_SIZE,
                                  label, color=label_color)

    # -- 框下方标注检测结果 --
    result_y = box_y + box_h + 4
    img.draw_string_advanced(2, result_y, FONT_SIZE,
                              "INT:%s" % inter.box_result,
                              color=GRAY_WHITE)
