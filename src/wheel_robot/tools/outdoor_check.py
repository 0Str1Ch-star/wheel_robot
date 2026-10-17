#!/usr/bin/env python3
"""室外赛场黄线识别 —— 离线检查工具。

直接用 shiwai.py 里真实的 DualYellowLineDetector / TrackSceneDetector，
参数优先读 config/shiwai.yaml，再套上 shiwai.py 顶部常量的覆盖
（和 ShiWaiFollower._apply_field_tuning 一致），所以测出来的就是车上实际生效的行为。

用法：
    # 1) 现场抓一帧（会等自动曝光收敛后再取，避免开头过曝）
    python3 src/wheel_robot/tools/outdoor_check.py --snap

    # 2) 分析已有的照片
    python3 src/wheel_robot/tools/outdoor_check.py --image ~/track.jpg

    # 3) 分析一段录像
    python3 src/wheel_robot/tools/outdoor_check.py \
        --video ~/wheel_robot/recordings/course_XXXX.avi --stride 6

    # 4) 临时改参数试效果（不改任何文件）
    python3 src/wheel_robot/tools/outdoor_check.py --image ~/track.jpg \
        --set yellow_s_min=30 yellow_v_min=50

输出：每帧一张对比图 + 终端遥测。
    <out>/<名字>_fNNNNNN.png      原图(带搜索框) | 识别结果(掩码+线+中心+场景框)
    顶部文字条给出「双边/单边/未识别」的判定，一眼就能看出好不好用。

看图的要点：
    右侧面板的 ROI 区域显示的是**黄线掩码**（黑白），不是原图。
      白块 = 被判成黄线的像素。黄线该是连续的一条；碎成点说明阈值不对。
    竖线 = 检测出的左右边界；十字圆圈 = 车道中心。
    原图面板上画的是搜索区域（search_top/bottom/left/right）。
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

import shiwai  # noqa: E402

PACKAGE_DIR = Path(__file__).resolve().parents[1]
CONFIG_YAML = PACKAGE_DIR / "config" / "shiwai.yaml"
DEFAULT_OUT = Path(__file__).resolve().parents[3] / "outdoor_check"

# 需要转成 int 的参数名（其余一律 float）
INT_KEYS = {
    "camera_id",
    "image_width",
    "image_height",
    "rotation_degrees",
    "yellow_h_min",
    "yellow_h_max",
    "yellow_s_min",
    "yellow_v_min",
    "morphology_kernel",
    "black_value_max",
    "black_saturation_max",
    "min_crosswalk_stripes",
}


# ----------------------------------------------------------------- 参数装载
def load_parameters() -> dict:
    """读 YAML 默认值，再套用 shiwai.py 顶部常量的覆盖（同 _apply_field_tuning）。"""
    params: dict = {}
    if CONFIG_YAML.exists():
        try:
            import yaml  # ROS 环境自带

            data = yaml.safe_load(CONFIG_YAML.read_text(encoding="utf-8")) or {}
            node = data.get("shiwai", {}).get("ros__parameters", {})
            params.update(node)
        except Exception as exc:  # noqa: BLE001
            print(f"⚠️  读 {CONFIG_YAML.name} 失败({exc})，改用代码内默认值")

    if not params:
        # YAML 缺失时的兜底，与 config/shiwai.yaml 保持一致
        params = {
            "search_top_ratio": 0.24,
            "search_bottom_ratio": 0.86,
            "search_left_ratio": 0.0,
            "search_right_ratio": 1.0,
            "yellow_h_min": 12,
            "yellow_h_max": 42,
            "yellow_s_min": 50,
            "yellow_v_min": 70,
            "min_boundary_lane_contrast": 18.0,
            "morphology_kernel": 3,
            "min_line_width_ratio": 0.012,
            "max_line_width_ratio": 0.50,
            "expected_line_width_ratio": 0.08,
            "width_tolerance_ratio": 0.08,
            "min_lane_width_ratio": 0.32,
            "max_lane_width_ratio": 0.82,
            "expected_lane_width_ratio": 0.72,
            "lane_width_tolerance_ratio": 0.30,
            "max_center_jump_ratio": 0.04,
            "min_valid_row_fraction": 0.12,
            "min_confidence": 0.38,
            "target_offset_pixels": 0.0,
            "black_value_max": 95,
            "black_saturation_max": 110,
            "crosswalk_top_ratio": 0.52,
            "crosswalk_bottom_ratio": 0.86,
            "hazard_top_ratio": 0.08,
            "hazard_bottom_ratio": 0.72,
            "side_margin_ratio": 0.03,
            "min_crosswalk_stripes": 4,
            "min_hazard_transitions": 1.8,
            "camera_backend": "v4l2",
            "camera_device": "/dev/video0",
            "image_width": 640,
            "image_height": 480,
            "capture_frame_rate": 30.0,
            "pixel_format": "YUYV",
        }

    # _apply_field_tuning() 会把这些从文件顶部常量重新赋值，这里照做
    params["search_top_ratio"] = float(shiwai.SEARCH_TOP_RATIO)
    params["search_bottom_ratio"] = float(shiwai.SEARCH_BOTTOM_RATIO)
    params["crosswalk_top_ratio"] = float(shiwai.CROSSWALK_TOP_RATIO)
    params["crosswalk_bottom_ratio"] = float(shiwai.CROSSWALK_BOTTOM_RATIO)
    params["min_crosswalk_stripes"] = int(shiwai.MIN_CROSSWALK_STRIPES)
    params["max_lane_width_ratio"] = 0.82
    params["target_offset_pixels"] = 0.0
    return params


def apply_overrides(params: dict, overrides: list[str]) -> dict:
    for item in overrides or []:
        if "=" not in item:
            raise SystemExit(f"--set 需要 key=value 形式：{item!r}")
        key, raw = item.split("=", 1)
        key = key.strip()
        if key not in params:
            raise SystemExit(f"未知参数 {key!r}（不在 shiwai 参数表里）")
        params[key] = int(float(raw)) if key in INT_KEYS else float(raw)
    return params


def build_detectors(params: dict):
    line = shiwai.DualYellowLineDetector(
        search_top_ratio=params["search_top_ratio"],
        search_bottom_ratio=params["search_bottom_ratio"],
        search_left_ratio=params["search_left_ratio"],
        search_right_ratio=params["search_right_ratio"],
        yellow_h_min=int(params["yellow_h_min"]),
        yellow_h_max=int(params["yellow_h_max"]),
        yellow_s_min=int(params["yellow_s_min"]),
        yellow_v_min=int(params["yellow_v_min"]),
        black_value_max=int(params["black_value_max"]),
        black_saturation_max=int(params["black_saturation_max"]),
        min_boundary_lane_contrast=params["min_boundary_lane_contrast"],
        morphology_kernel=int(params["morphology_kernel"]),
        min_line_width_ratio=params["min_line_width_ratio"],
        max_line_width_ratio=params["max_line_width_ratio"],
        expected_line_width_ratio=params["expected_line_width_ratio"],
        width_tolerance_ratio=params["width_tolerance_ratio"],
        min_lane_width_ratio=params["min_lane_width_ratio"],
        max_lane_width_ratio=params["max_lane_width_ratio"],
        expected_lane_width_ratio=params["expected_lane_width_ratio"],
        lane_width_tolerance_ratio=params["lane_width_tolerance_ratio"],
        max_center_jump_ratio=params["max_center_jump_ratio"],
        min_valid_row_fraction=params["min_valid_row_fraction"],
        min_confidence=params["min_confidence"],
        target_offset_pixels=params["target_offset_pixels"],
    )
    scene = shiwai.TrackSceneDetector(
        yellow_h_min=int(params["yellow_h_min"]),
        yellow_h_max=int(params["yellow_h_max"]),
        yellow_s_min=int(params["yellow_s_min"]),
        yellow_v_min=int(params["yellow_v_min"]),
        black_value_max=int(params["black_value_max"]),
        black_saturation_max=int(params["black_saturation_max"]),
        crosswalk_top_ratio=params["crosswalk_top_ratio"],
        crosswalk_bottom_ratio=params["crosswalk_bottom_ratio"],
        hazard_top_ratio=params["hazard_top_ratio"],
        hazard_bottom_ratio=params["hazard_bottom_ratio"],
        side_margin_ratio=params["side_margin_ratio"],
        min_crosswalk_stripes=int(params["min_crosswalk_stripes"]),
        min_hazard_transitions=params["min_hazard_transitions"],
    )
    return line, scene


def measure_lane_width(frame: np.ndarray, params: dict) -> dict:
    """逐行量出真实车道宽度（像素），并给出 max/min/expected_lane_width_ratio 建议。

    这三个 ratio 都是相对 ROI 宽度的比例，写死后很难猜。这里用真实画面量：
    每一行取「最左合规游程」和「最右合规游程」的内沿距，即车道宽。
    同时会按 min_boundary_lane_contrast 过滤掉“黄线不是真的黄线”的行。
    """
    det, _ = build_detectors(params)
    height, width = frame.shape[:2]
    x0 = int(width * params["search_left_ratio"])
    x1 = int(width * params["search_right_ratio"])
    y0 = int(height * params["search_top_ratio"])
    y1 = int(height * params["search_bottom_ratio"])
    roi = frame[y0:y1, x0:x1]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = det._yellow_mask(hsv)
    kernel_size = int(params["morphology_kernel"])
    if kernel_size > 1:
        kernel = np.ones((kernel_size, kernel_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    roi_h, roi_w = mask.shape
    min_line = max(2, int(roi_w * params["min_line_width_ratio"]))
    max_line = max(min_line + 1, int(roi_w * params["max_line_width_ratio"]))
    contrast_min = params["min_boundary_lane_contrast"]

    widths: list[float] = []
    rejected = 0
    for row_y in range(roi_h - 1, -1, -1):
        runs = [
            (start, end, end - start)
            for start, end in det._runs(mask[row_y])
            if min_line <= end - start <= max_line
        ]
        if len(runs) < 2:
            continue
        left, right = runs[0], runs[-1]
        pair_width = float(right[0]) - float(left[1] - 1)
        if pair_width <= 0:
            continue
        corridor = hsv[row_y, left[1] : right[0], 1]
        if corridor.size == 0:
            continue
        boundary_sat = 0.5 * (
            float(np.mean(hsv[row_y, left[0] : left[1], 1]))
            + float(np.mean(hsv[row_y, right[0] : right[1], 1]))
        )
        if boundary_sat - float(np.median(corridor)) < contrast_min:
            rejected += 1
            continue
        widths.append(pair_width)

    if not widths:
        return {"ok": False, "rejected": rejected, "roi_w": roi_w, "roi_h": roi_h}
    arr = np.array(widths)
    return {
        "ok": True,
        "rejected": rejected,
        "roi_w": roi_w,
        "roi_h": roi_h,
        "n": int(arr.size),
        "min": float(arr.min()),
        "p10": float(np.percentile(arr, 10)),
        "median": float(np.median(arr)),
        "p90": float(np.percentile(arr, 90)),
        "max": float(arr.max()),
    }


def print_measure(stat: dict) -> None:
    print("\n=== 车道宽度实测（像素）===")
    print(f"ROI {stat['roi_w']}x{stat['roi_h']}")
    if not stat["ok"]:
        print(f"❌ 没有一行能凑出合规左右游程（对比度不足被否 {stat['rejected']} 行）。")
        print("   先把 yellow_h_* / yellow_s_min 调到能稳定出掩码，再回来测宽度。")
        return
    roi_w = stat["roi_w"]
    print(f"有效行 {stat['n']}  对比度不足被否 {stat['rejected']} 行")
    print(f"  最小 {stat['min']:.0f}  p10 {stat['p10']:.0f}  中位 {stat['median']:.0f} "
          f"p90 {stat['p90']:.0f}  最大 {stat['max']:.0f}")
    print("\n建议写进 config/shiwai.yaml：")
    print(f"  expected_lane_width_ratio: {stat['median'] / roi_w:.2f}"
          f"    # 中位 {stat['median']:.0f}px / {roi_w}")
    print(f"  min_lane_width_ratio: {(stat['p10'] * 0.85) / roi_w:.2f}"
          f"    # p10 {stat['p10']:.0f}px 再留 15% 余量")
    print(f"  max_lane_width_ratio: {min(0.98, stat['max'] * 1.15 / roi_w):.2f}"
          f"    # 最大 {stat['max']:.0f}px 再留 15% 余量")
    print("\n（当前值："
          "min 0.32 / max 0.82 / expected 0.72 —— 如果差别很大，"
          "说明这份配置和你的实际赛道/机位对不上。）")


# ----------------------------------------------------------------- 现场抓帧
def grab_snapshot(
    params: dict,
    max_warmup: int = 180,
    settle_tol: float = 3.0,
    settle_need: int = 10,
) -> np.ndarray:
    """从摄像头抓一帧。

    不能用固定帧数：摄像头刚上电时自动曝光(AE)还没收敛，开头一两秒整幅画面
    是白的（实测录像第 0 帧 V=255、标准差 0；第 60 帧仍有 69% 纯白像素，
    到第 150 帧才正常）。所以这里**等亮度稳定**再取帧。
    """
    backend = str(params.get("camera_backend", "v4l2")).lower()
    device = str(params.get("camera_device", "/dev/video0"))
    cap = (
        cv2.VideoCapture(device, cv2.CAP_V4L2)
        if backend == "v4l2"
        else cv2.VideoCapture(int(params.get("camera_id", 0)))
    )
    if not cap.isOpened():
        raise SystemExit(
            f"打不开摄像头 {device}。若巡线节点还在跑，先停掉（同一个设备只能一个进程用）。"
        )
    if backend == "v4l2":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(params.get("image_width", 640)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(params.get("image_height", 480)))
    cap.set(cv2.CAP_PROP_FPS, float(params.get("capture_frame_rate", 30.0)))

    frame = None
    prev_mean = None
    stable = 0
    used = 0
    for index in range(max_warmup):
        ok, got = cap.read()
        if not ok:
            continue
        used = index + 1
        frame = got
        # 只看亮度均值，够判断 AE 是否收敛，比全量统计便宜很多
        mean_v = float(cv2.cvtColor(got, cv2.COLOR_BGR2HSV)[:, :, 2].mean())
        if prev_mean is not None and abs(mean_v - prev_mean) <= settle_tol:
            stable += 1
            if stable >= settle_need:
                break
        else:
            stable = 0
        prev_mean = mean_v
    cap.release()

    if frame is None:
        raise SystemExit("摄像头没读到帧。")

    height, width = frame.shape[:2]
    v = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)[:, :, 2]
    mean_v = float(v.mean())
    blown = float((v >= 250).mean())
    print(
        f"抓到一帧 {width}x{height}：读 {used} 帧后亮度稳定（V均值 {mean_v:.0f}，"
        f"接近全白 {blown * 100:.0f}%）"
    )
    if stable < settle_need:
        print(f"⚠️  {max_warmup} 帧内亮度没稳定，场景可能本身就忽明忽暗。")
    if blown > 0.30 or mean_v > 230:
        print(
            "⚠️  画面明显过曝（阳光直射或 AE 到顶了），HSV 门限会失效。"
            "\n    处理办法："
            "\n      1) 给镜头遮一下直射阳光 / 压低机位俯角"
            "\n      2) 固定曝光（先查控件名）："
            "\n         v4l2-ctl -d /dev/video0 --list-ctrls"
            "\n         v4l2-ctl -d /dev/video0 -c auto_exposure=1 "
            "-c exposure_time_absolute=<值>"
        )
    return frame


# ----------------------------------------------------------------- 单帧分析
def analyse(frame: np.ndarray, detectors) -> tuple:
    line_det, scene_det = detectors
    result = line_det.detect(frame)
    # 场景检测直接画到巡线返回的 debug_frame 上，一张图能看到全部结果
    scene = scene_det.detect(frame, result.debug_frame)
    return result, scene


def verdict(result, scene) -> tuple[str, tuple[int, int, int]]:
    """给出人话判定。注意：只能用 ASCII —— cv2.putText 不支持中文。"""
    left, right = result.left_line_rows, result.right_line_rows
    if not result.valid:
        return f"FAIL: {result.reason}", (80, 80, 255)
    if left > 0 and right > 0:
        return "OK: both edges found", (80, 255, 80)
    if left > 0 or right > 0:
        side = "LEFT" if left > 0 else "RIGHT"
        return f"WARN: {side} edge only -> center inferred", (60, 200, 255)
    return "WARN: valid but no edge rows", (60, 200, 255)


def compose(frame: np.ndarray, result, scene, params: dict, tag: str) -> np.ndarray:
    h, w = frame.shape[:2]
    left = frame.copy()

    # 左面板：画搜索区域，说明检测器到底在看哪里
    x0 = int(round(w * params["search_left_ratio"]))
    x1 = int(round(w * params["search_right_ratio"])) - 1
    y0 = int(round(h * params["search_top_ratio"]))
    y1 = int(round(h * params["search_bottom_ratio"])) - 1
    cv2.rectangle(left, (x0, y0), (x1, y1), (255, 180, 0), 2)
    cv2.putText(
        left,
        "SEARCH AREA",
        (x0 + 6, max(16, y0 - 6)),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.5,
        (255, 180, 0),
        2,
        cv2.LINE_AA,
    )
    right = result.debug_frame

    text_h = 132
    canvas = np.full((h + text_h, w * 2, 3), 18, dtype=np.uint8)
    canvas[text_h : text_h + h, 0:w] = left
    canvas[text_h : text_h + h, w : w * 2] = right

    word, color = verdict(result, scene)
    lines = [
        (
            f"{tag}   H {params['yellow_h_min']}-{params['yellow_h_max']}  "
            f"S>={params['yellow_s_min']}  V>={params['yellow_v_min']}  "
            f"k={params['morphology_kernel']}  minConf={params['min_confidence']}",
            (200, 200, 200),
        ),
        (
            f"valid={result.valid}  reason={result.reason}  "
            f"conf={result.confidence:.2f}  lane_w={result.lane_width_pixels:.0f}px",
            (230, 230, 230),
        ),
        (
            f"rows: left={result.left_line_rows}  right={result.right_line_rows}  "
            f"err={result.error:+.3f}",
            (230, 230, 230),
        ),
        (
            f"crosswalk={int(scene.crosswalk)}({scene.crosswalk_score:.2f},"
            f"{scene.stripe_count}st)  hazard={int(scene.hazard)}"
            f"({scene.hazard_score:.2f})  finish={int(scene.finish)}"
            f"({scene.finish_score:.2f},{scene.finish_blobs}blob)",
            (230, 230, 230),
        ),
        (word, color),
    ]
    for index, (text, tone) in enumerate(lines):
        cv2.putText(
            canvas,
            text,
            (10, 24 + 24 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.52,
            tone,
            1,
            cv2.LINE_AA,
        )
    cv2.putText(canvas, "RAW + SEARCH AREA", (10, h + text_h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
    cv2.putText(canvas, "YELLOW MASK + EDGES + CENTER + SCENE BOXES",
                (w + 10, h + text_h - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1, cv2.LINE_AA)
    return canvas


# ----------------------------------------------------------------- 主流程
def iter_video(path: Path, start: int, stride: int, limit: int | None):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频：{path}")
    index = 0
    emitted = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if index >= start and (index - start) % stride == 0:
                yield index, frame
                emitted += 1
                if limit is not None and emitted >= limit:
                    break
            index += 1
    finally:
        cap.release()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="室外赛黄线识别离线检查（用 shiwai.py 的真实检测器与参数）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--snap", action="store_true", help="现场抓一帧并分析")
    source.add_argument("--image", type=Path, help="分析一张图片")
    source.add_argument("--video", type=Path, help="分析一段录像")
    parser.add_argument("--start", type=int, default=40, help="视频起始帧（跳过暖机）")
    parser.add_argument("--stride", type=int, default=6, help="每 N 帧取 1 帧")
    parser.add_argument("--limit", type=int, default=40, help="最多分析多少帧")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT, help="输出目录")
    parser.add_argument("--set", nargs="*", default=[], metavar="KEY=VAL",
                        help="临时覆盖参数，如 --set yellow_s_min=30")
    parser.add_argument("--measure", action="store_true",
                        help="额外实测车道像素宽度，并给出三个 ratio 的建议值")
    args = parser.parse_args()

    params = apply_overrides(load_parameters(), args.set)
    detectors = build_detectors(params)
    args.out.mkdir(parents=True, exist_ok=True)

    print(
        f"参数：H {params['yellow_h_min']}-{params['yellow_h_max']}  "
        f"S>={params['yellow_s_min']}  V>={params['yellow_v_min']}  "
        f"搜索 {params['search_top_ratio']:.2f}~{params['search_bottom_ratio']:.2f}  "
        f"人行道 0.{int(params['crosswalk_top_ratio']*100)}~"
        f"{params['crosswalk_bottom_ratio']:.2f}  "
        f"最少{params['min_crosswalk_stripes']}条"
    )

    if args.snap:
        frame = grab_snapshot(params)
        if args.measure:
            print_measure(measure_lane_width(frame, params))
        result, scene = analyse(frame, detectors)
        tag = "snap"
        canvas = compose(frame, result, scene, params, tag)
        raw_path = args.out / f"{tag}_raw.png"
        out_path = args.out / f"{tag}.png"
        cv2.imwrite(str(raw_path), frame)
        cv2.imwrite(str(out_path), canvas)
        word, _ = verdict(result, scene)
        print(f"\n{word}")
        print(f"  reason={result.reason} conf={result.confidence:.3f} "
              f"左行={result.left_line_rows} 右行={result.right_line_rows} "
              f"车道宽={result.lane_width_pixels:.0f}px")
        print(f"  人行道={scene.crosswalk}({scene.crosswalk_score:.2f},"
              f"{scene.stripe_count}条) 黄黑={scene.hazard}"
              f"({scene.hazard_score:.2f}) 终点={scene.finish}"
              f"({scene.finish_score:.2f})")
        print(f"\n原图   {raw_path}")
        print(f"对比图 {out_path}")
        return 0

    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            raise SystemExit(f"读不到图片：{args.image}")
        if args.measure:
            print_measure(measure_lane_width(frame, params))
        result, scene = analyse(frame, detectors)
        canvas = compose(frame, result, scene, params, args.image.stem)
        out_path = args.out / f"{args.image.stem}.png"
        cv2.imwrite(str(out_path), canvas)
        word, _ = verdict(result, scene)
        print(f"\n{word}   conf={result.confidence:.2f}  "
              f"左行={result.left_line_rows} 右行={result.right_line_rows}")
        print(f"对比图 {out_path}")
        return 0

    total = ok_valid = both = single = none = 0
    for index, frame in iter_video(args.video, args.start, args.stride, args.limit):
        result, scene = analyse(frame, detectors)
        canvas = compose(frame, result, scene, params, f"f{index:06d}")
        cv2.imwrite(str(args.out / f"{args.video.stem}_f{index:06d}.png"), canvas)
        total += 1
        if result.valid:
            ok_valid += 1
            if result.left_line_rows > 0 and result.right_line_rows > 0:
                both += 1
            else:
                single += 1
        else:
            none += 1

    if not total:
        raise SystemExit("没读到任何帧，检查 --start / 视频文件。")
    print(f"\n共 {total} 帧")
    print(f"  有效       {ok_valid:>4}  {ok_valid/total*100:>5.0f}%")
    print(f"    双边都有 {both:>4}  {both/total*100:>5.0f}%   ← 这个高才说明黄线识别好用")
    print(f"    只有单边 {single:>4}  {single/total*100:>5.0f}%")
    print(f"  未识别     {none:>4}  {none/total*100:>5.0f}%")
    print(f"\n对比图在 {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
