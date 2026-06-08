#!/usr/bin/env python3
"""VL53L8CX 8x8 ToF heatmap visualization node using OpenCV."""

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Float32MultiArray

GRID = 8
CELL_PX = 80          # 각 셀 픽셀 크기
MAX_DIST = 4000.0      # mm 최대 거리 (컬러맵 범위)
MIN_DIST = 0.0

# tof_bridge ROI 파라미터 (실제 파라미터에서 확인됨)
ROI_ROW_START = 2     # roi_rows = [2, 5] → Row 2,3,4
ROI_ROW_END = 5
ROI_COL_START = 2     # roi_cols = [2, 5] → Col 2,3,4
ROI_COL_END = 5

# 창 크기
MAP_W = CELL_PX * GRID
MAP_H = CELL_PX * GRID
INFO_H = 140           # 하단 정보 영역
WIN_W = MAP_W
WIN_H = MAP_H + INFO_H


class TofHeatmapNode(Node):
    def __init__(self):
        super().__init__('tof_heatmap')

        self.grid = np.full((GRID, GRID), np.nan, dtype=np.float64)
        self.min_val = 0.0
        self.max_val = 0.0
        self.roi_val = 0.0        # ROI 영역 평균 (mm)
        self.front_distance = 0.0  # tof_bridge가 계산한 실제 값 (m)

        self.create_subscription(
            Float32MultiArray, '/tof/distances', self._cb_distances, 10
        )
        self.create_subscription(
            Float32, '/tof/front_distance', self._cb_front, 10
        )

        # 30 FPS 렌더링
        self.create_timer(1.0 / 30.0, self._render)

        cv2.namedWindow('VL53L8CX 8x8 ToF Heatmap', cv2.WINDOW_AUTOSIZE)
        self.get_logger().info('ToF Heatmap node started')

    def _cb_front(self, msg):
        self.front_distance = msg.data

    def _cb_distances(self, msg):
        data = np.array(msg.data, dtype=np.float64)
        if len(data) == GRID * GRID:
            self.grid = data.reshape((GRID, GRID))
            valid = self.grid[np.isfinite(self.grid)]
            if len(valid) > 0:
                self.min_val = float(np.min(valid))
                self.max_val = float(np.max(valid))
            # ROI 영역 평균 (tof_bridge와 동일한 영역)
            roi = self.grid[ROI_ROW_START:ROI_ROW_END, ROI_COL_START:ROI_COL_END]
            roi_valid = roi[np.isfinite(roi)]
            self.roi_val = float(np.mean(roi_valid)) if len(roi_valid) > 0 else 0.0

    def _render(self):
        canvas = np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8)

        # --- 히트맵 생성 ---
        norm = self.grid.copy()
        nan_mask = ~np.isfinite(norm)
        norm[nan_mask] = MAX_DIST
        norm = np.clip(norm, MIN_DIST, MAX_DIST)
        # 가까울수록 빨강(Hot), 멀수록 파랑(Cold) → COLORMAP_JET 반전
        norm_u8 = (255 - (norm / MAX_DIST * 255)).astype(np.uint8)
        heatmap_small = cv2.applyColorMap(norm_u8, cv2.COLORMAP_JET)
        # NaN 셀은 검정
        for r in range(GRID):
            for c in range(GRID):
                if nan_mask[r, c]:
                    heatmap_small[r, c] = [30, 30, 30]

        heatmap = cv2.resize(heatmap_small, (MAP_W, MAP_H), interpolation=cv2.INTER_NEAREST)
        canvas[0:MAP_H, 0:MAP_W] = heatmap

        # --- 격자선 + 거리값 텍스트 ---
        for r in range(GRID):
            for c in range(GRID):
                x1 = c * CELL_PX
                y1 = r * CELL_PX
                # 격자선
                cv2.rectangle(canvas, (x1, y1), (x1 + CELL_PX, y1 + CELL_PX),
                              (60, 60, 60), 1)
                # 거리값 (mm)
                val = self.grid[r, c]
                if np.isfinite(val):
                    txt = f'{int(val)}'
                    # 밝기에 따라 텍스트 색상 결정
                    brightness = norm_u8[r, c]
                    text_color = (0, 0, 0) if brightness > 128 else (255, 255, 255)
                    font_scale = 0.5
                    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, font_scale, 1)
                    tx = x1 + (CELL_PX - tw) // 2
                    ty = y1 + (CELL_PX + th) // 2
                    cv2.putText(canvas, txt, (tx, ty),
                                cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_color, 1,
                                cv2.LINE_AA)
                else:
                    txt = 'NaN'
                    (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1)
                    tx = x1 + (CELL_PX - tw) // 2
                    ty = y1 + (CELL_PX + th) // 2
                    cv2.putText(canvas, txt, (tx, ty),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1,
                                cv2.LINE_AA)

        # --- ROI 영역 강조 (tof_bridge의 실제 ROI: R2-4, C2-4) ---
        roi_x1 = ROI_COL_START * CELL_PX
        roi_y1 = ROI_ROW_START * CELL_PX
        roi_x2 = ROI_COL_END * CELL_PX
        roi_y2 = ROI_ROW_END * CELL_PX
        cv2.rectangle(canvas, (roi_x1, roi_y1), (roi_x2, roi_y2), (0, 255, 255), 2)
        cv2.putText(canvas, 'ROI (front_distance)', (roi_x1 + 4, roi_y1 - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1, cv2.LINE_AA)

        # --- 행/열 라벨 ---
        for r in range(GRID):
            y = r * CELL_PX + CELL_PX // 2 + 4
            cv2.putText(canvas, f'R{r}', (2, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)
        for c in range(GRID):
            x = c * CELL_PX + CELL_PX // 2 - 8
            cv2.putText(canvas, f'C{c}', (x, 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

        # --- 하단 정보 ---
        info_y = MAP_H + 10
        cv2.putText(canvas, 'VL53L8CX 8x8 ToF Heatmap', (10, info_y + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'Min: {self.min_val:.0f}mm  Max: {self.max_val:.0f}mm',
                    (10, info_y + 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'ROI avg: {self.roi_val:.0f}mm ({self.roi_val/1000:.2f}m)',
                    (10, info_y + 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(canvas, f'front_distance (actual): {self.front_distance:.3f}m',
                    (10, info_y + 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)

        # 컬러바
        bar_x = 10
        bar_y = info_y + 105
        bar_w = MAP_W - 20
        bar_h = 15
        for i in range(bar_w):
            val = int(255 - (i / bar_w * 255))
            color = cv2.applyColorMap(np.array([[val]], dtype=np.uint8), cv2.COLORMAP_JET)[0][0]
            cv2.line(canvas, (bar_x + i, bar_y), (bar_x + i, bar_y + bar_h),
                     color.tolist(), 1)
        cv2.putText(canvas, '0m', (bar_x, bar_y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
        cv2.putText(canvas, f'{MAX_DIST/1000:.0f}m', (bar_x + bar_w - 25, bar_y - 3),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        cv2.imshow('VL53L8CX 8x8 ToF Heatmap', canvas)
        key = cv2.waitKey(1)
        if key == 27:  # ESC
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = TofHeatmapNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
