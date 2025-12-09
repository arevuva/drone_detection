"""
Streaming drone detector for ESP32-CAM feeds using YOLOv8.

Usage:
  python esp32_stream_detect.py --stream-url http://<esp32-ip>:81/stream --weights best.pt --show

The script connects to the MJPEG stream served by the ESP32-CAM, runs
YOLOv8 inference frame-by-frame, and prints a textual verdict with the
current drone probability to the console. Optionally, it can also render
an OpenCV window with the detections overlaid.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Optional

import cv2
from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Drone detection on ESP32-CAM video stream.")
    parser.add_argument(
        "--stream-url",
        type=str,
        default="http://192.168.4.1:81/stream",
        help="MJPEG stream URL exposed by the ESP32-CAM (default matches CameraWebServer example).",
    )
    parser.add_argument(
        "--weights",
        type=str,
        default="best.pt",
        help="Path to YOLOv8 weights (best.pt already included in the repo).",
    )
    parser.add_argument(
        "--conf",
        type=float,
        default=0.35,
        help="Confidence threshold for YOLO predictions.",
    )
    parser.add_argument(
        "--iou",
        type=float,
        default=0.5,
        help="IoU threshold for NMS / tracker (higher → fewer boxes).",
    )
    parser.add_argument(
        "--imgsz",
        type=int,
        default=640,
        help="Inference image size for YOLO (resizes internally, e.g. 320 for 320x240 streams).",
    )
    parser.add_argument(
        "--device",
        type=str,
        default=None,
        help="Inference device (e.g. 'cpu', '0'); if None, ultralytics auto-selects.",
    )
    parser.add_argument(
        "--print-interval",
        type=float,
        default=1.0,
        help="Seconds between console updates to avoid spamming logs.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display an OpenCV window with detections drawn.",
    )
    parser.add_argument(
        "--use-track",
        action="store_true",
        help="Use YOLO tracking (ByteTrack) instead of plain predict to reduce flicker/false positives.",
    )
    parser.add_argument(
        "--save-dir",
        type=str,
        default=None,
        help="Folder to save frames with detected drones (annotated with boxes). Disabled if not set.",
    )
    parser.add_argument(
        "--serial-port",
        type=str,
        default=None,
        help="If set, send detection status over this serial port to an ESP32 display (e.g. /dev/ttyUSB0).",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baudrate for serial telemetry to ESP32 (when --serial-port is set).",
    )
    parser.add_argument(
        "--serial-interval",
        type=float,
        default=1.0,
        help="Minimum seconds between serial messages to ESP32.",
    )
    return parser.parse_args()


def load_model(weights_path: Path, device: Optional[str]) -> YOLO:
    if not weights_path.exists():
        sys.exit(f"Weights file not found: {weights_path}")
    try:
        return YOLO(str(weights_path)).to(device if device else None)
    except Exception as exc:  # pragma: no cover - relies on external libs
        sys.exit(f"Failed to load YOLO model from {weights_path}: {exc}")


def connect_stream(url: str) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        sys.exit(f"Cannot open stream at {url}. Check ESP32 connectivity and URL.")
    # Reduce latency for MJPEG streams.
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap


def extract_drone_conf(result) -> Optional[float]:
    """
    Returns the highest confidence for class 'drone' in a YOLO result, if present.
    """
    boxes = getattr(result, "boxes", None)
    if boxes is None:
        return None

    max_conf: Optional[float] = None
    for box in boxes:
        cls_id = int(box.cls)
        cls_name = result.names.get(cls_id, str(cls_id)).lower()
        if cls_name != "drone":
            continue
        conf = float(box.conf)
        if max_conf is None or conf > max_conf:
            max_conf = conf
    return max_conf


def main() -> None:
    args = parse_args()
    weights_path = Path(args.weights)
    save_dir = Path(args.save_dir) if args.save_dir else None
    if save_dir:
        save_dir.mkdir(parents=True, exist_ok=True)

    ser = None
    if args.serial_port:
        try:
            import serial  # type: ignore
        except ImportError:
            sys.exit("pyserial is required for --serial-port. Install with: pip install pyserial")
        try:
            ser = serial.Serial(args.serial_port, args.baud, timeout=0)
            print(f"Serial telemetry enabled on {args.serial_port} @ {args.baud}")
        except Exception as exc:  # pragma: no cover - depends on hardware
            sys.exit(f"Failed to open serial port {args.serial_port}: {exc}")

    model = load_model(weights_path, args.device)
    cap = connect_stream(args.stream_url)

    last_print = 0.0
    last_state: Optional[bool] = None
    last_serial = 0.0
    print("Connected to stream. Press Ctrl+C or close the window to stop.")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from stream. Re-check the connection.")
                time.sleep(0.5)
                continue

            if args.use_track:
                results = model.track(
                    frame,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device=args.device,
                    iou=args.iou,
                    persist=True,  # keep tracker state across frames
                    verbose=False,
                )
            else:
                results = model.predict(
                    frame,
                    imgsz=args.imgsz,
                    conf=args.conf,
                    device=args.device,
                    iou=args.iou,
                    verbose=False,
                )
            drone_conf: Optional[float] = None
            if results:
                drone_conf = extract_drone_conf(results[0])

            now = time.time()
            state = drone_conf is not None
            should_print = now - last_print >= args.print_interval or state != last_state
            if should_print:
                if drone_conf is None:
                    print(f"[{time.strftime('%H:%M:%S')}] Drone: NOT detected")
                else:
                    print(
                        f"[{time.strftime('%H:%M:%S')}] Drone detected | probability {drone_conf*100:.2f}%"
                    )
                last_print = now
                last_state = state

            if ser and now - last_serial >= args.serial_interval:
                if drone_conf is None:
                    msg = "NONE\n"
                else:
                    msg = f"DRONE {drone_conf:.3f}\n"
                try:
                    ser.write(msg.encode("utf-8"))
                except Exception:
                    pass  # best-effort telemetry
                last_serial = now

            if args.show:
                frame_to_show = results[0].plot() if results else frame
                cv2.imshow("ESP32-CAM Drone Detection", frame_to_show)
                if cv2.waitKey(1) & 0xFF == 27:  # ESC
                    break

            # Save annotated frames when a drone is detected.
            if save_dir and drone_conf is not None and results:
                annotated = results[0].plot()  # includes bounding boxes/labels
                ts_ms = int(time.time() * 1000)
                out_path = save_dir / f"drone_{ts_ms}.jpg"
                cv2.imwrite(str(out_path), annotated)
                print(f"Saved detection frame to {out_path}")
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
