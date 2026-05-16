"""
路口检测模块

算法:
  在图像上半区域中间放一排检测框 → 统计每框黑像素占比
  → 过半则标记为 1 → 连续多框为 1 → 判定路口

  (左右边缘各留 80px 边距, 因为路口横线不铺满全屏)

帧格式 (0x02):
  | 0xAA | 0x02 | intersection_type(1B) | direction(1B) | XOR |
  - type: 0=无, 1=十字, 2=T字左, 3=T字右
  - direction: 0=无, 1=左转, 2=右转, 3=直行
"""

from detector_base import Detector


class IntersectionDetector(Detector):
    """路口检测器 — 网格黑像素统计 (仅中间有效区域)"""

    NUM_BOXES        = 6         # 检测框数量
    BOX_LEFT_MARGIN  = 80        # 左边缘跳过 (px)
    BOX_RIGHT_MARGIN = 80        # 右边缘跳过 (px)
    BOX_HEIGHT       = 12        # 每框高度 (px)
    BOX_Y_OFFSET     = 40        # 框排起始 y (距顶部)
    BLACK_FILL_RATIO = 0.5       # 黑像素过半 → 框置 1
    MIN_ACTIVE_BOXES = 3         # 最少连续活动框数 → 判定路口
    CONFIRM_FRAMES   = 3         # 多帧确认

    def __init__(self):
        super().__init__(name="Intersection")
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0
        self.box_active = [0] * self.NUM_BOXES   # 供可视化
        self.box_result = ""                      # 供可视化

    # ================================================================
    # 基类方法重写
    # ================================================================

    def Init(self):
        super().Init()
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0
        self.box_active = [0] * self.NUM_BOXES
        self.box_result = ""

    def Process(self, img, left_pts, right_pts):
        """
        在图像上半区中间放置一排检测框，统计黑像素判定路口

        参数:
            img       : 已二值化的灰度图 (0=黑/路面, 255=白/车道线)
            left_pts  : 未使用 (保持接口兼容)
            right_pts : 未使用 (保持接口兼容)

        返回:
            (type, direction) or None
        """
        w = img.width()
        h = img.height()

        # 有效区域: 左右各留边距
        x0_region = self.BOX_LEFT_MARGIN
        x1_region = w - self.BOX_RIGHT_MARGIN
        region_w  = x1_region - x0_region
        box_w     = region_w // self.NUM_BOXES
        box_y     = self.BOX_Y_OFFSET
        box_h     = self.BOX_HEIGHT

        # ---- 逐框统计黑像素 ----
        active = [0] * self.NUM_BOXES
        for i in range(self.NUM_BOXES):
            x0 = x0_region + i * box_w
            x1 = min(x0 + box_w, x1_region)
            black_cnt = 0
            total = 0
            for y in range(box_y, min(box_y + box_h, h)):
                for x in range(x0, x1):
                    total += 1
                    if img.get_pixel(x, y) < 128:
                        black_cnt += 1

            if total > 0 and black_cnt / total >= self.BLACK_FILL_RATIO:
                active[i] = 1

        self.box_active = active

        # ---- 找最长连续活动段 ----
        max_run = 0
        cur_run = 0
        for v in active:
            if v == 1:
                cur_run += 1
            else:
                if cur_run > max_run:
                    max_run = cur_run
                cur_run = 0
        if cur_run > max_run:
            max_run = cur_run

        # ---- 判定类型 ----
        if max_run < self.MIN_ACTIVE_BOXES:
            itype = 0
        else:
            left_active  = sum(active[:self.NUM_BOXES // 3])
            right_active = sum(active[2 * self.NUM_BOXES // 3:])
            total_active = sum(active)

            if total_active >= self.NUM_BOXES * 0.7:
                itype = 1          # 十字: 大部分框都黑
            elif left_active > right_active:
                itype = 2          # T字左
            else:
                itype = 3          # T字右

        # ---- 多帧确认 ----
        return self._Confirm(itype)

    def Reset(self):
        self._result = {"type": 0, "direction": 0}
        self._confirm_count = 0
        self._last_type     = 0
        self.box_active = [0] * self.NUM_BOXES
        self.box_result = ""

    def SendToUart(self):
        """发送路口检测结果"""
        if self._uart is not None and self._result["type"] > 0:
            self._uart.SendIntersection(
                self._result["type"],
                self._result["direction"]
            )
            TYPE_NAMES = {1: "CROSS", 2: "T-LEFT", 3: "T-RIGHT"}
            print("[TX] Inter | type=%d(%s) dir=%d"
                  % (self._result["type"],
                     TYPE_NAMES.get(self._result["type"], "?"),
                     self._result["direction"]))

    # ================================================================
    # 多帧确认
    # ================================================================

    def _Confirm(self, itype):
        TYPE_NAMES = {0: "-", 1: "CROSS", 2: "T-LEFT", 3: "T-RIGHT"}
        self.box_result = TYPE_NAMES.get(itype, "?")

        if itype > 0 and itype == self._last_type:
            self._confirm_count += 1
        else:
            self._confirm_count = 0

        self._last_type = itype

        if self._confirm_count >= self.CONFIRM_FRAMES:
            direction = {1: 3, 2: 1, 3: 2}.get(itype, 0)
            self._result = {"type": itype, "direction": direction}
            self._confirm_count = 0
            return (itype, direction)

        return None
