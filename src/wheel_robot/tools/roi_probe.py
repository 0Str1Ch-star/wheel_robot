#!/usr/bin/env python3
"""ROI 探针：用一段干净录像判断 ROI_TOP_RATIO 该取多少。

做两件事：
  1. 亮度剖面 —— 逐行统计「亮白像素占比」，找出背景干扰（看台/白墙）的下边界。
  2. 方案对比 —— 用 dual_line_follower.DualWhiteLineDetector 真跑一遍，
     比较几个候选 ROI_TOP_RATIO 的有效率 / 分段数 / 置信度 / far 段质量。

  3. 自动丢弃「空白帧」——摄像头刚打开时前若干帧是纯白的，混进来会
     把整个剖面算成「全画面过曝」，得出完全相反的结论。

用法：
    # 先录一段干净 640x480（不要用带调试叠加的 480x240 合成图）
    ros2 run wheel_robot camera_recorder.py

    python3 src/wheel_robot/tools/roi_probe.py ~/wheel_robot/recordings/course_XXXX.avi

    # 只看剖面，不跑检测器（快）
    python3 src/wheel_robot/tools/roi_probe.py <video> --profile-only

注意：ROI_TOP_RATIO 是模块常量不是 ROS 参数，改完必须 colcon build。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

import dual_line_follower as dl  # noqa: E402

# 「亮白」判定：饱和度低 + 亮度高。看台、白墙、反光地面都会落进来。
WHITE_LIGHT_V_MIN = 140
# 亮白占比低于这个值，就认为该行已经是纯跑道、没有背景干扰。
CLEAN_RATIO = 0.02


def iter_frames(path: Path, stride: int, start: int, limit: int | None):
    """按顺序读帧（不用 cap.set 随机定位，MJPEG 随机 seek 会解码失败）。"""
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频：{path}")
    idx = 0
    emitted = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx >= start and (idx - start) % stride == 0:
                yield idx, frame
                emitted += 1
                if limit is not None and emitted >= limit:
                    break
            idx += 1
    finally:
        cap.release()


def drop_blank_frames(frames: list[np.ndarray]) -> tuple[list[np.ndarray], int]:
    """丢掉暖机空帧/亮度塌掉的坏帧。

    摄像头刚上电的前若干帧是纯白（V=255 且标准差 0）。若混进剖面统计，
    会被当成「背景亮白占比 100%」，得出「全画面过曝」的错误结论。
    """
    kept: list[np.ndarray] = []
    blank = 0
    for frame in frames:
        v = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
        if float(np.mean(v >= 250)) > 0.97 and float(np.std(v)) < 5.0:
            blank += 1
        else:
            kept.append(frame)
    return kept, blank


def check_geometry(frames: list[np.ndarray]) -> bool:
    """调试图是 480x240 合成图，不能喂给检测器。"""
    h, w = frames[0].shape[:2]
    if (w, h) == (640, 480):
        return True
    print(f"⚠️  画面 {w}x{h}，不是干净的 640x480。")
    print("    如果这是 line_follower/dual_line_follower 输出的调试录屏，")
    print("    左边是原始画面、右边是 ROI/MASK 叠加，检测器结果不可信。")
    print("    请用 camera_recorder.py 重录。\n")
    return False


def brightness_profile(frames: list[np.ndarray]) -> list[float]:
    """返回逐行的亮白像素占比（长度 = 画面高度）。"""
    h = frames[0].shape[0]
    acc = np.zeros(h, dtype=np.float64)
    for frame in frames:
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = (hsv[:, :, 1] <= dl.WHITE_S_MAX) & (hsv[:, :, 2] >= WHITE_LIGHT_V_MIN)
        acc += mask.mean(axis=1)
    return (acc / len(frames)).tolist()


def print_profile(profile: list[float]) -> float | None:
    """打印亮度剖面，返回「背景干扰下边界」。

    注意：这里的亮白判据是宽松的绝对阈值（S<=WHITE_S_MAX 且 V>=140），会把
    远处天空、亮路面也算进来。它只说明「画面上部有没有大块亮背景」，
    不能单独决定 ROI_TOP_RATIO —— 真正约束是切掉后远视野还剩多少，
    那必须看检测器实测（见 compare_detector）。
    """
    h = len(profile)
    print("① 亮度剖面（亮白像素占该行比例，每 5% 一档）")
    print(f"   {'画面位置':>10}  {'亮白占比':>8}   图示")
    print("   " + "-" * 46)
    for pct in range(0, 101, 5):
        row = min(h - 1, int(h * pct / 100))
        ratio = profile[row]
        bar = "█" * int(round(ratio * 30))
        print(f"   {pct:>8}%  {ratio:>8.3f}   {bar}")

    # 从下往上找：最低的「变干净」转折点，就是建议的 ROI 上边。
    # 从底向上扫，一旦遇到脏行就停，取它上面一行。
    clean_start = None
    for row in range(h - 1, -1, -1):
        if profile[row] > CLEAN_RATIO:
            clean_start = row + 1
            break
    if clean_start is None:
        print("\n   整幅画面都很干净 → 没有大块亮背景干扰。")
        return 0.0
    if clean_start >= h:
        print("\n   ❌ 全画面都偏亮，看不出干净的跑道 —— 可能过曝或机位不对。")
        return None

    boundary = clean_start / h
    print(f"\n   背景干扰下边界 ≈ 画面 {boundary * 100:.0f}%（第 {clean_start} 行）")
    print("   （仅供参考：此判据偏宽松，最终取值以 ② 的检测器实测为准）")
    if boundary > 0.60:
        print("   ⚠️  亮背景占了画面上部 60% 以上，切掉后远视野会很紧张。")
        print("       若 ② 显示高 ROI 的 far 质量断崖下跌，应先调摄像头俯仰角，")
        print("       而不是继续抬高 ROI 硬躲。")
    return round(boundary, 2)


def compare_detector(frames: list[np.ndarray], tops: list[float]) -> list[dict]:
    """按不同 ROI_TOP_RATIO 真跑一遍检测器，返回每档的指标。"""
    print("\n② 各 ROI_TOP_RATIO 下的检测效果（同一个检测器，只换 ROI）")
    header = (f"   {'ROI顶':>6} {'有效':>9} {'≥2段':>8} {'≥3段':>8} "
              f"{'far有效':>8} {'conf':>7} {'far质量':>8}")
    print(header)
    print("   " + "-" * (len(header) - 3))

    rows: list[dict] = []
    for top in tops:
        dl.ROI_TOP_RATIO = top
        detector = dl.DualWhiteLineDetector()
        total = valid = seg2 = seg3 = far_ok = 0
        confs: list[float] = []
        far_q: list[float] = []
        for frame in frames:
            result = detector.detect(frame)
            total += 1
            if result.valid:
                valid += 1
            segments = sum(
                1 for seg in (result.near, result.mid, result.far) if seg.valid
            )
            if segments >= 2:
                seg2 += 1
            if segments >= 3:
                seg3 += 1
            if result.far.valid:
                far_ok += 1
            confs.append(result.confidence)
            far_q.append(result.far.quality if result.far.valid else 0.0)
        if not total:
            continue
        row = {
            "top": top,
            "valid": valid / total,
            "seg2": seg2 / total,
            "seg3": seg3 / total,
            "far_ok": far_ok / total,
            "conf": float(np.mean(confs)),
            "far_q": float(np.mean(far_q)),
        }
        rows.append(row)
        print(f"   {top:>6.2f} {valid:>4}/{total:<4} {seg2:>4} {seg2/total*100:>3.0f}%"
              f" {seg3:>4} {seg3/total*100:>3.0f}% {far_ok/total*100:>7.0f}%"
              f" {row['conf']:>7.3f} {row['far_q']:>8.3f}")

    print("\n   读法：「far质量」是关键指标 —— 背景干扰若污染 far 段，它会先掉。")
    print("        「≥3段」掉了说明 ROI 切太狠、远视野不够了。")
    return rows


def print_recommendation(rows: list[dict]) -> None:
    """基于检测器实测指标给出取值建议，并标出断崖位置。"""
    print("\n③ 建议")
    if not rows:
        print("   没有有效数据。")
        return

    best = max(rows, key=lambda r: (r["far_q"], r["conf"]))
    spread = max(r["far_q"] for r in rows) - min(r["far_q"] for r in rows)
    print(f"   far 质量最高：ROI_TOP_RATIO = {best['top']:.2f}"
          f"（far质量 {best['far_q']:.3f}，conf {best['conf']:.3f}）")

    # 断崖：比最优点更高、但 far 质量跌掉 30% 以上，说明远视野被切没了。
    # 但曲线常常非单调（局部场景差异 + 样本噪声），所以先看下跌是否持续。
    above = sorted((r for r in rows if r["top"] > best["top"]),
                   key=lambda r: r["top"])
    bad = [r for r in above if r["far_q"] < best["far_q"] * 0.7]
    if bad:
        first_bad = bad[0]["top"]
        recovered = [r for r in above
                     if r["top"] > first_bad and r["far_q"] >= best["far_q"] * 0.9]
        if recovered:
            print(f"   注意：{first_bad:.2f} 处 far 质量掉到 {bad[0]['far_q']:.3f}，"
                  f"但 {recovered[0]['top']:.2f} 又回到 {recovered[0]['far_q']:.3f}。")
            print("        曲线非单调（局部场景 + 样本噪声），别当成硬边界；"
                  "加大 --limit 复测更稳。")
        else:
            print(f"   ⚠️  {first_bad:.2f} 之后 far 质量持续偏低"
                  f" → 实际取值别超过 {first_bad - 0.02:.2f}")

    if spread < 0.05:
        print("   各取值几乎无差别 → 这段录像里没有明显干扰，")
        print("   要评估看台影响请换到有看台的机位重录。")
        return

    print(f"   改法：scripts/dual_line_follower.py 顶部常量"
          f" ROI_TOP_RATIO = {best['top']:.2f}")
    print("   然后 colcon build --packages-select wheel_robot"
          "（常量不能用 -p 覆盖）。")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="判断 ROI_TOP_RATIO 该取多少（看台/白墙干扰探针）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("video", type=Path, help="干净录像路径（640x480）")
    parser.add_argument("--start", type=int, default=40,
                        help="起始帧号，默认 40（跳过摄像头暖机空帧）")
    parser.add_argument("--stride", type=int, default=6, help="每 N 帧取 1 帧")
    parser.add_argument("--limit", type=int, default=80, help="最多取多少帧")
    parser.add_argument("--tops", type=float, nargs="+", default=None,
                        help="要对比的 ROI_TOP_RATIO 列表，默认在 0.34~0.66 扫一遍")
    parser.add_argument("--profile-only", action="store_true",
                        help="只输出亮度剖面，不跑检测器")
    args = parser.parse_args()

    frames = [f for _, f in iter_frames(args.video, args.stride, args.start, args.limit)]
    if not frames:
        raise SystemExit(f"没读到任何帧：{args.video}")
    print(f"读取 {args.video.name}：{len(frames)} 帧"
          f"（stride={args.stride}, start={args.start}）")

    frames, blank = drop_blank_frames(frames)
    if blank:
        print(f"已丢弃 {blank} 帧空白帧（摄像头暖机/坏帧）")
    if not frames:
        raise SystemExit("剩余帧全是空白帧，换一段录像（或加大 --start）。")
    print()

    ok = check_geometry(frames)
    profile = brightness_profile(frames)
    suggested = print_profile(profile)

    if args.profile_only:
        return 0

    if not ok:
        print("\n跳过检测器对比（画面尺寸不对）。")
        return 0

    tops = args.tops
    if tops is None:
        # 扫常用区间：低于 0.34 没意义，高于 0.66 远视野基本被切没。
        tops = [round(0.34 + 0.04 * i, 2) for i in range(9)]  # 0.34 ~ 0.66
        if suggested:
            tops = sorted(set(tops + [suggested]))
    rows = compare_detector(frames, tops)
    print_recommendation(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main())
