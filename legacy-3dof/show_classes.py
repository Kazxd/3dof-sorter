"""
Show what each numeric class ('0'..'5') in your dataset actually is.

It reads the YOLO label files, crops a few example objects per class and stitches
them into  class_preview/class_<id>.jpg .  Open those images, decide the real
name of each class, then fill in CLASS_NAMES in app.py.

    python show_classes.py
"""
import glob
import os
import cv2
import numpy as np

DATASET_DIR = "Nuts and Bolts.v1i.yolov8"   # folder that contains train/ valid/ data.yaml
SPLIT = "train"
SAMPLES_PER_CLASS = 8
TILE = 160                                   # size of each cropped thumbnail (px)

os.makedirs("class_preview", exist_ok=True)
samples = {}                                 # class id -> list of crops

for lbl_path in glob.glob(os.path.join(DATASET_DIR, SPLIT, "labels", "*.txt")):
    stem = os.path.splitext(os.path.basename(lbl_path))[0]
    img_path = next(iter(glob.glob(os.path.join(DATASET_DIR, SPLIT, "images", stem + ".*"))), None)
    if img_path is None:
        continue
    img = cv2.imread(img_path)
    if img is None:
        continue
    h, w = img.shape[:2]
    for line in open(lbl_path):
        parts = line.split()
        if len(parts) < 5:
            continue
        cid = int(parts[0])
        if len(samples.get(cid, [])) >= SAMPLES_PER_CLASS:
            continue
        cx, cy, bw, bh = (float(p) for p in parts[1:5])   # normalised YOLO format
        x1, y1 = int((cx - bw / 2) * w), int((cy - bh / 2) * h)
        x2, y2 = int((cx + bw / 2) * w), int((cy + bh / 2) * h)
        crop = img[max(0, y1):y2, max(0, x1):x2]
        if crop.size:
            samples.setdefault(cid, []).append(cv2.resize(crop, (TILE, TILE)))

for cid in sorted(samples):
    sheet = np.hstack(samples[cid])
    cv2.putText(sheet, f"class {cid}", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    out = f"class_preview/class_{cid}.jpg"
    cv2.imwrite(out, sheet)
    print("saved", out, f"({len(samples[cid])} samples)")
