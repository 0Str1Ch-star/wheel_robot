#!/usr/bin/env python3
"""离线帧检查工具：从录像截一段帧，导出【ROI + HSV 通道 + 白线掩码】。

这是**开发调试工具**，不参与车上运行。它直接 import line_follower.py，
复用真实的 OutdoorWhiteLineDetector 和全部阈值常量，所以看到的掩码
和车上跑出来的一模一样。

用法示例：
  # 第 0~1 秒，每 5 帧取一张，输出到 /tmp/mask_test
  python3 tools/inspect_frames.py ~/wheel_robot/recordings/course_xxx.avi \
      --start 0.0 --duration 1.0 --stride 5 --out /tmp/mask_test

  # 想让图大一点在电脑上看
  python3 tools/inspect_frames.py course_xxx.avi --scale 2.0

输出（默认写到 ~/wheel_robot/mask_debug/）：
  <out>_f000000_t0.000.png   每帧一张四联图（ROI | V 通道 | S 通道 | MASK）
  <out>_contact.png          所有采样帧的 MASK 拼成一张联络表
  <out>_raw_f000000_*.png    --save-raw 时额外保存完整 640x480 原始帧
  终端打印                    每帧的 dyn_v / 白像素占比 / 各判定门的通过像素数
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

# 直接复用 wheel_robot 包里的实现，避免两套阈值不一致
SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import line_follower as lf  # noqa: E402

# 默认输出目录：放在工作区下，避免 /tmp 重启被清空
DEFAULT_OUTPUT_DIR = Path(__file__).resolve().parents[3] / "mask_debug"

SEGMENTS = (
    ("FAR", lf.FAR_RANGE, (255, 180, 0)),   # BGR -> 天蓝
    ("MID", lf.MID_RANGE, (0, 220, 255)),   # BGR -> 橙黄
    ("NEAR", lf.NEAR_RANGE, (0, 255, 0)),   # BGR -> 绿
)


def crop_roi(frame: np.ndarray) -> np.ndarray:
    """按 line_follower 的规则裁 ROI 并缩放到 PROCESS_WIDTH x PROCESS_HEIGHT。"""
    height, width = frame.shape[:2]
    x0 = int(round(width * lf.ROI_LEFT_RATIO))
    x1 = int(round(width * lf.ROI_RIGHT_RATIO))
    y0 = int(round(height * lf.ROI_TOP_RATIO))
    y1 = int(round(height * lf.ROI_BOTTOM_RATIO))
    roi = frame[y0:y1, x0:x1]
    return cv2.resize(
        roi,
        (lf.PROCESS_WIDTH, lf.PROCESS_HEIGHT),
        interpolation=cv2.INTER_AREA,
    )


def annotate(canvas: np.ndarray) -> np.ndarray:
    """画三段分界线 + 中心线(白) + 控制参考线(青)，与车上调试窗一致。"""
    width, height = lf.PROCESS_WIDTH, lf.PROCESS_HEIGHT
    for name, (lo, _hi), color in SEGMENTS:
        y = int(round(lo * height))
        cv2.line(canvas, (0, y), (width - 1, y), color, 1)
        cv2.putText(canvas, name, (4, max(12, y + 12)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.36, color, 1, cv2.LINE_AA)
    cv2.line(canvas, (width // 2, 0), (width // 2, height - 1),
             (255, 255, 255), 1)
    cx = int(round(width / 2.0 + lf.LINE_TARGET_OFFSET_PX))
    cv2.line(canvas, (cx, 0), (cx, height - 1), (255, 255, 0), 1)
    return canvas


def gray3(img: np.ndarray) -> np.ndarray:
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def gate_report(roi: np.ndarray, dyn_v: int) -> dict:
    """统计各判定门单独通过的像素数，用来定位"是哪一条卡住了白线"。"""
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    _h, s, v = cv2.split(hsv)
    block = lf.ADAPTIVE_BLOCK_SIZE
    if block % 2 == 0:
        block += 1
    low_sat = s <= lf.WHITE_S_MAX
    sample = v[low_sat]
    local = cv2.adaptiveThreshold(
        v, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,
        block, lf.ADAPTIVE_C,
    ) > 0
    strong = v >= min(255, dyn_v + lf.STRONG_WHITE_V_MARGIN)
    return {
        "total": int(v.size),
        "low_sat": int(np.count_nonzero(low_sat)),
        "absolute": int(np.count_nonzero(v >= dyn_v)),
        "local": int(np.count_nonzero(local)),
        "strong": int(np.count_nonzero(strong)),
        "v_p50": float(np.percentile(sample, 50)) if sample.size else 0.0,
        "v_p82": float(np.percentile(sample, lf.DYNAMIC_V_PERCENTILE))
        if sample.size else 0.0,
        "v_p95": float(np.percentile(sample, 95)) if sample.size else 0.0,
    }


def build_panel(
    frame: np.ndarray,
    detector: "lf.OutdoorWhiteLineDetector",
    label: str,
) -> tuple[np.ndarray, int, float, dict]:
    """返回 (四联图, dyn_v, white_ratio, 各门统计)。"""
    roi = crop_roi(frame)
    mask, dyn_v = detector._make_mask(roi)
    white_ratio = float(cv2.countNonZero(mask)) / float(mask.size)

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    _h, s, v = cv2.split(hsv)

    roi_panel = annotate(roi.copy())
    v_panel = annotate(gray3(v))
    s_panel = annotate(gray3(s))
    mask_panel = annotate(gray3(mask))

    for panel, title in (
        (roi_panel, "ROI BGR"),
        (v_panel, "V (brightness)"),
        (s_panel, "S (saturation)"),
        (mask_panel, "MASK"),
    ):
        cv2.putText(panel, title, (lf.PROCESS_WIDTH - 118, 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1,
                    cv2.LINE_AA)

    top = np.hstack((roi_panel, v_panel))
    bottom = np.hstack((s_panel, mask_panel))
    panel = np.vstack((top, bottom))

    cv2.putText(
        panel,
        f"{label}  dyn_v={dyn_v}  white={white_ratio:.5f}",
        (6, panel.shape[0] - 8),
        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1, cv2.LINE_AA,
    )
    return panel, dyn_v, white_ratio, gate_report(roi, dyn_v)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video", help="干净录像路径（camera_recorder.py 录的）")
    parser.add_argument("--start", type=float, default=0.0, help="起始秒，默认 0")
    parser.add_argument("--duration", type=float, default=1.0, help="截取时长秒，默认 1.0")
    parser.add_argument("--stride", type=int, default=5, help="每 N 帧取一张，默认 5")
    parser.add_argument(
        "--out",
        default=str(DEFAULT_OUTPUT_DIR / "mask"),
        help=f"输出文件名前缀，默认 {DEFAULT_OUTPUT_DIR}/mask",
    )
    parser.add_argument("--scale", type=float, default=1.0, help="输出放大倍数，默认 1.0")
    parser.add_argument(
        "--save-raw",
        action="store_true",
        help="额外保存完整 640x480 原始帧（默认只存四联图）",
    )
    args = parser.parse_args()

    out_prefix = Path(args.out).expanduser()
    out_prefix.parent.mkdir(parents=True, exist_ok=True)

    video_path = Path(args.video).expanduser()
    if not video_path.exists():
        raise SystemExit(f"找不到视频: {video_path}")

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频: {video_path}")

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    print(f"视频: {video_path.name}")
    print(f"  {width}x{height} @ {fps:.1f} fps, {total} 帧, "
          f"{total / fps:.1f} 秒")
    if (width, height) != (lf.IMAGE_WIDTH, lf.IMAGE_HEIGHT):
        print(f"  ⚠️ 分辨率不是 {lf.IMAGE_WIDTH}x{lf.IMAGE_HEIGHT}，"
              "可能不是 camera_recorder 录的干净视频，结果会失真")

    # 当前生效的关键阈值，一起打印出来方便对照
    print("\n当前生效的关键阈值：")
    for name in ("DYNAMIC_V_PERCENTILE", "DYNAMIC_V_OFFSET", "DYNAMIC_V_MIN",
                 "DYNAMIC_V_MAX", "WHITE_S_MAX", "ADAPTIVE_BLOCK_SIZE",
                 "ADAPTIVE_C", "STRONG_WHITE_V_MARGIN",
                 "LINE_TARGET_OFFSET_PX"):
        print(f"  {name:26s} = {getattr(lf, name)}")

    start_frame = int(round(args.start * fps))
    end_frame = int(round((args.start + args.duration) * fps))
    end_frame = min(end_frame, total)
    stride = max(1, args.stride)

    detector = lf.OutdoorWhiteLineDetector()
    masks = []
    print(f"\n采样帧 {start_frame} ~ {end_frame}（每 {stride} 帧取一张）")
    print(f"{'帧':>7} {'秒':>6} {'dyn_v':>6} {'white%':>9} "
          f"{'低饱和':>8} {'绝对亮':>8} {'局部亮':>8} {'极亮':>7} "
          f"{'V50':>5} {'V82':>5} {'V95':>5}")

    for frame_id in range(start_frame, end_frame, stride):
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        ok, frame = cap.read()
        if not ok:
            break
        t = frame_id / fps
        label = f"frame {frame_id}  t={t:.3f}s"
        panel, dyn_v, white_ratio, rep = build_panel(frame, detector, label)

        if args.scale != 1.0:
            panel = cv2.resize(panel, None, fx=args.scale, fy=args.scale,
                               interpolation=cv2.INTER_AREA)
        cv2.imwrite(f"{out_prefix}_f{frame_id:06d}_t{t:.3f}.png", panel)
        if args.save_raw:
            cv2.imwrite(
                f"{out_prefix}_raw_f{frame_id:06d}_t{t:.3f}.png", frame
            )

        masks.append(crop_roi_panel(panel, args.scale))
        print(f"{frame_id:7d} {t:6.3f} {dyn_v:6d} {white_ratio * 100:8.3f}% "
              f"{rep['low_sat']:8d} {rep['absolute']:8d} {rep['local']:8d} "
              f"{rep['strong']:7d} {rep['v_p50']:5.0f} {rep['v_p82']:5.0f} "
              f"{rep['v_p95']:5.0f}")

    cap.release()

    if masks:
        sheet = np.vstack(masks)
        sheet_path = Path(f"{args.out}_contact.png")
        cv2.imwrite(str(sheet_path), sheet)
        print(f"\n✅ 单帧四联图: {out_prefix}_f*_t*.png")
        if args.save_raw:
            print(f"✅ 原始帧:     {out_prefix}_raw_f*_t*.png")
        print(f"✅ 掩码联络表: {sheet_path}")


def crop_roi_panel(panel: np.ndarray, scale: float) -> np.ndarray:
    """从四联图里裁出 MASK 那一格（右下 1/4），用于拼联络表。"""
    h, w = panel.shape[:2]
    cell = int(round(lf.PROCESS_HEIGHT * scale))
    half_w = int(round(lf.PROCESS_WIDTH * scale))
    return panel[h - cell:h, w - half_w:w].copy()


if __name__ == "__main__":
    main()
