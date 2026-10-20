"""
calibrate_camera.py - build the pixel -> millimetre homography for the table.

    python calibrate_camera.py            click the reference points (default)
    python calibrate_camera.py --aruco    use ArUco markers listed in ARUCO_MARKERS_MM
    python calibrate_camera.py --verify   show the saved calibration; hover to read mm

Before you start
  * Phone on the tripod, final height, lens parallel to the table. Lock focus and
    exposure in the IP Webcam app. Pick the stream resolution you will run with.
  * Mark the points in config.CALIB_POINTS_MM on the mat (printed crosses work well)
    and measure each one from the base axis with a ruler. 4 is the minimum;
    6-9 spread over the mat average out click error.

Keys (click mode)
  left click  place the next point (a zoom box helps you hit the centre)
  u           undo last point          r   restart
  f           freeze / unfreeze the frame (clicking on a still image is easier)
  s           save (once all points are placed)          q / Esc  quit
"""
import argparse
import sys
import time

import cv2
import numpy as np

import config as C
from camera import FrameGrabber
from mapping import PlaneMapper, detect_aruco_centres, mapper_from_aruco

WIN = "calibrate"
mouse = {"x": -1, "y": -1, "clicks": []}


def on_mouse(event, x, y, flags, param):
    mouse["x"], mouse["y"] = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        mouse["clicks"].append((x, y))


def wait_frame(grabber, timeout=15.0):
    end = time.time() + timeout
    while time.time() < end:
        frame, _ = grabber.latest()
        if frame is not None:
            return frame
        time.sleep(0.05)
    sys.exit(f"No frames from {grabber.source}. Check CAMERA_URL / the phone app.")


def draw_grid(img, mapper, step=20.0):
    """Project a millimetre grid over the pick area so you can check it against a ruler."""
    x0, x1 = C.WORKSPACE_X[0] - 40, C.WORKSPACE_X[1] + 40
    y0, y1 = C.WORKSPACE_Y[0] - 40, C.WORKSPACE_Y[1] + 40
    for x in np.arange(np.floor(x0 / step) * step, x1 + 1, step):
        p0, p1 = mapper.mm_to_px(x, y0), mapper.mm_to_px(x, y1)
        cv2.line(img, tuple(map(int, p0)), tuple(map(int, p1)), (0, 255, 0) if x % 100 else (0, 255, 255), 1)
    for y in np.arange(np.floor(y0 / step) * step, y1 + 1, step):
        p0, p1 = mapper.mm_to_px(x0, y), mapper.mm_to_px(x1, y)
        cv2.line(img, tuple(map(int, p0)), tuple(map(int, p1)), (0, 255, 0) if y % 100 else (0, 255, 255), 1)
    cv2.polylines(img, [mapper.workspace_polygon_px()], True, (255, 0, 255), 2)


def draw_zoom(img, src, x, y, half=20, scale=5):
    """Magnified view of the area under the cursor, top-right corner."""
    h, w = src.shape[:2]
    if not (0 <= x < w and 0 <= y < h):
        return
    x0, y0 = max(0, x - half), max(0, y - half)
    crop = src[y0:min(h, y + half), x0:min(w, x + half)]
    if crop.size == 0:
        return
    z = cv2.resize(crop, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    cx, cy = (x - x0) * scale + scale // 2, (y - y0) * scale + scale // 2
    cv2.line(z, (cx, 0), (cx, z.shape[0]), (0, 0, 255), 1)
    cv2.line(z, (0, cy), (z.shape[1], cy), (0, 0, 255), 1)
    zh, zw = z.shape[:2]
    if zw < w and zh < h:
        img[0:zh, w - zw:w] = z
        cv2.rectangle(img, (w - zw, 0), (w - 1, zh - 1), (255, 255, 255), 1)


def text(img, s, row, color=(255, 255, 255)):
    cv2.putText(img, s, (10, 24 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, (10, 24 + 22 * row), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)


# =============================================================================
def click_mode(grabber):
    targets = C.CALIB_POINTS_MM
    if len(targets) < 4:
        sys.exit("config.CALIB_POINTS_MM needs at least 4 points")
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)
    frozen = None
    mapper = None
    while True:
        live = wait_frame(grabber)
        src = frozen if frozen is not None else live
        img = src.copy()
        h, w = img.shape[:2]
        clicks = mouse["clicks"]
        del clicks[len(targets):]

        for i, (u, v) in enumerate(clicks):
            cv2.drawMarker(img, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
            cv2.putText(img, f"P{i + 1} {targets[i]}", (u + 8, v - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        if len(clicks) == len(targets):
            try:
                mapper = PlaneMapper.from_points(clicks, targets, (w, h))
                draw_grid(img, mapper)
                text(img, f"fit rms error {mapper.rms_mm:.2f} mm - grid should sit on your marks. "
                          f"s = save, u = undo", 0, (0, 255, 0))
            except ValueError as e:
                mapper = None
                text(img, f"fit failed: {e}", 0, (0, 0, 255))
        else:
            k = len(clicks)
            text(img, f"click P{k + 1} at X={targets[k][0]:.0f} Y={targets[k][1]:.0f} mm "
                      f"({k}/{len(targets)})", 0)
        text(img, "u undo  r restart  f freeze  s save  q quit" + ("   [FROZEN]" if frozen is not None else ""), 1)
        draw_zoom(img, src, mouse["x"], mouse["y"])
        cv2.imshow(WIN, img)

        key = cv2.waitKey(20) & 0xFF
        if key in (ord("q"), 27):
            return None
        if key == ord("u") and clicks:
            clicks.pop()
        elif key == ord("r"):
            clicks.clear()
        elif key == ord("f"):
            frozen = None if frozen is not None else live.copy()
        elif key == ord("s") and mapper is not None:
            mapper.save(extra={"points_px": [list(map(float, c)) for c in clicks],
                               "points_mm": [list(t) for t in targets]})
            print(f"saved {C.HOMOGRAPHY_FILE}  (rms {mapper.rms_mm:.2f} mm, image {w}x{h})")
            return mapper


def aruco_mode(grabber):
    cv2.namedWindow(WIN)
    while True:
        frame = wait_frame(grabber)
        img = frame.copy()
        centres = detect_aruco_centres(frame)
        for i, (u, v) in centres.items():
            ok = i in C.ARUCO_MARKERS_MM
            cv2.circle(img, (int(u), int(v)), 6, (0, 255, 0) if ok else (0, 0, 255), 2)
            cv2.putText(img, f"id {i}", (int(u) + 8, int(v)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        mapper = mapper_from_aruco(frame)
        if mapper is not None:
            draw_grid(img, mapper)
            text(img, f"{len(centres)} markers, rms {mapper.rms_mm:.2f} mm - s = save, q = quit", 0, (0, 255, 0))
        else:
            text(img, f"need 4 of ids {sorted(C.ARUCO_MARKERS_MM)} - seeing {sorted(centres)}", 0, (0, 0, 255))
        cv2.imshow(WIN, img)
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            return None
        if key == ord("s") and mapper is not None:
            mapper.save()
            print(f"saved {C.HOMOGRAPHY_FILE}  (rms {mapper.rms_mm:.2f} mm)")
            return mapper


def verify_mode(grabber):
    if not C.HOMOGRAPHY_FILE.exists():
        sys.exit("No calibration saved yet.")
    mapper = PlaneMapper.load()
    cv2.namedWindow(WIN)
    cv2.setMouseCallback(WIN, on_mouse)
    print("Hover over ruler marks and compare. q to quit.")
    while True:
        frame = wait_frame(grabber)
        h, w = frame.shape[:2]
        if not mapper.set_frame_size(w, h):
            print("WARNING: aspect ratio differs from the calibration image - recalibrate")
        img = frame.copy()
        draw_grid(img, mapper)
        if 0 <= mouse["x"] < w and 0 <= mouse["y"] < h:
            x, y = mapper.px_to_mm(mouse["x"], mouse["y"])
            text(img, f"cursor  X={x:7.1f}  Y={y:7.1f} mm", 0, (0, 255, 255))
        base = mapper.mm_to_px(0, 0)
        if 0 <= base[0] < w and 0 <= base[1] < h:
            cv2.drawMarker(img, tuple(map(int, base)), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.imshow(WIN, img)
        if cv2.waitKey(30) & 0xFF in (ord("q"), 27):
            return


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--camera", help="override CAMERA_URL")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--aruco", action="store_true")
    g.add_argument("--verify", action="store_true")
    args = p.parse_args()
    grabber = FrameGrabber(args.camera)
    grabber.start()
    try:
        if args.verify:
            verify_mode(grabber)
        elif args.aruco:
            aruco_mode(grabber)
        else:
            click_mode(grabber)
    finally:
        grabber.stop()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
