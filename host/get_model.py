"""
get_model.py - get a washer / nut / bolt YOLOv8 model.

There is no ready-made .pt file for these three classes that you can download and
trust on your table, so this script builds one in about 15-30 minutes on a GPU
(Google Colab's free T4 is fine) from public Roboflow datasets + your own images.

    set ROBOFLOW_API_KEY=xxxx          (Windows; free key: roboflow.com -> Settings -> API)
    python get_model.py download       public datasets -> ../datasets/raw/
    python get_model.py merge          remap every class name to washer/nut/bolt -> ../datasets/merged/
    python get_model.py train          fine-tune yolov8n -> models/best.pt
    python get_model.py all            all three

Your own photos (strongly recommended): capture with collect_images.py, label the
three classes in Roboflow or CVAT, export "YOLOv8", unzip into ../datasets/own/
(it must contain data.yaml). merge picks it up automatically and repeats it
OWN_REPEAT times in the training split so the model learns your setup first.

Datasets used (CC BY 4.0, check each page for the latest version number):
  jingjing-huang/yellow_silver   ~1.4k top-down images with ArUco markers;
                                 classes {silver, yellow} x {bolt, nut, washer}
  tktk-hodc3/nutsnbolts          50 images, classes nut / bolt / washer
Search https://universe.roboflow.com for "class:washer class:nut class:bolt"
to add more; anything whose class names contain washer / nut / bolt / screw merges.
"""
import argparse
import os
import shutil
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = HERE.parent / "datasets"
RAW, MERGED, OWN = DATA / "raw", DATA / "merged", DATA / "own"
MODEL_OUT = HERE / "models" / "best.pt"

SOURCES = [
    dict(name="yellow_silver", workspace="jingjing-huang", project="yellow_silver", version=4),
    dict(name="nutsnbolts", workspace="tktk-hodc3", project="nutsnbolts", version=5),
]
OWN_REPEAT = 3
CLASSES = ["washer", "nut", "bolt"]          # index = class id in the merged dataset
SPLITS = {"train": "train", "valid": "valid", "val": "valid", "test": "test"}
IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


# =============================================================================
def download():
    key = os.environ.get("ROBOFLOW_API_KEY")
    if not key:
        sys.exit("Set ROBOFLOW_API_KEY first (free account at roboflow.com -> Settings -> API Keys).")
    try:
        from roboflow import Roboflow
    except ImportError:
        sys.exit("pip install roboflow")
    rf = Roboflow(api_key=key)
    RAW.mkdir(parents=True, exist_ok=True)
    for s in SOURCES:
        dest = RAW / s["name"]
        if (dest / "data.yaml").exists():
            print(f"[skip] {s['name']} already downloaded")
            continue
        url = f"https://universe.roboflow.com/{s['workspace']}/{s['project']}"
        try:
            print(f"[download] {s['name']} v{s['version']} from {url}")
            rf.workspace(s["workspace"]).project(s["project"]).version(s["version"]).download(
                "yolov8", location=str(dest))
        except Exception as e:
            print(f"[error] {s['name']}: {e}\n        check the version number on {url} and edit SOURCES")


# =============================================================================
def read_names(yaml_path):
    import yaml
    with open(yaml_path, "r", encoding="utf-8") as f:
        d = yaml.safe_load(f)
    names = d.get("names", [])
    if isinstance(names, dict):
        return {int(k): v for k, v in names.items()}
    return dict(enumerate(names))


def merge():
    import vision                              # reuses the same name -> label rules as the app
    sources = [p for p in sorted(RAW.glob("*")) if (p / "data.yaml").exists()]
    if (OWN / "data.yaml").exists():
        sources.append(OWN)
    if not sources:
        sys.exit("Nothing to merge: run 'download' and/or put your export in datasets/own/")
    if MERGED.exists():
        shutil.rmtree(MERGED)

    counts = Counter()
    for src in sources:
        names = read_names(src / "data.yaml")
        remap = {}
        for cid, name in names.items():
            label = vision.canonical_label(name)
            if label:
                remap[cid] = CLASSES.index(label)
        print(f"[merge] {src.name}: " + ", ".join(
            f"{names[c]}->{CLASSES[remap[c]] if c in remap else 'DROP'}" for c in names))
        repeat = OWN_REPEAT if src == OWN else 1

        for split_dir in src.iterdir():
            split = SPLITS.get(split_dir.name)
            if not split or not (split_dir / "images").is_dir():
                continue
            out_img, out_lbl = MERGED / split / "images", MERGED / split / "labels"
            out_img.mkdir(parents=True, exist_ok=True)
            out_lbl.mkdir(parents=True, exist_ok=True)
            for img in (split_dir / "images").iterdir():
                if img.suffix.lower() not in IMG_EXT:
                    continue
                lbl = split_dir / "labels" / (img.stem + ".txt")
                lines = []
                if lbl.exists():
                    for line in lbl.read_text(encoding="utf-8").splitlines():
                        parts = line.split()
                        if len(parts) < 5:
                            continue
                        cid = int(float(parts[0]))
                        if cid not in remap:
                            continue
                        if len(parts) > 5:          # polygon label -> bounding box
                            xs, ys = list(map(float, parts[1::2])), list(map(float, parts[2::2]))
                            cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2
                            bw, bh = max(xs) - min(xs), max(ys) - min(ys)
                            parts = [parts[0], f"{cx:.6f}", f"{cy:.6f}", f"{bw:.6f}", f"{bh:.6f}"]
                        lines.append(" ".join([str(remap[cid])] + parts[1:5]))
                        counts[(split, CLASSES[remap[cid]])] += 1 if split != "train" else repeat
                for r in range(repeat if split == "train" else 1):
                    stem = f"{src.name}_{r}_{img.stem}"[:150]          # keep paths short for Windows
                    shutil.copy2(img, out_img / (stem + img.suffix.lower()))
                    (out_lbl / (stem + ".txt")).write_text("\n".join(lines), encoding="utf-8")

    val_dir = "valid" if (MERGED / "valid").exists() else "train"
    yaml_text = (f"path: {MERGED.as_posix()}\ntrain: train/images\nval: {val_dir}/images\n"
                 + ("test: test/images\n" if (MERGED / "test").exists() else "")
                 + "names:\n" + "".join(f"  {i}: {n}\n" for i, n in enumerate(CLASSES)))
    (MERGED / "data.yaml").write_text(yaml_text, encoding="utf-8")
    print("[merge] boxes per split/class:")
    for (split, label), n in sorted(counts.items()):
        print(f"   {split:6s} {label:7s} {n}")
    print(f"[merge] wrote {MERGED / 'data.yaml'}")


# =============================================================================
def train(model_name="yolov8n.pt", epochs=80, imgsz=640, batch=16, device=None):
    try:
        from ultralytics import YOLO
    except ImportError:
        sys.exit("pip install ultralytics")
    data = MERGED / "data.yaml"
    if not data.exists():
        sys.exit("Run 'merge' first.")
    model = YOLO(model_name)
    model.train(
        data=str(data), epochs=epochs, imgsz=imgsz, batch=batch, patience=20, device=device,
        project=str(HERE / "runs" / "train"), name="fasteners", exist_ok=True,
        # top-down camera: any rotation / flip is a real possibility, perspective is not
        degrees=180, fliplr=0.5, flipud=0.5, perspective=0.0,
        hsv_v=0.4, close_mosaic=10,
    )
    best = Path(model.trainer.best)
    MODEL_OUT.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(best, MODEL_OUT)
    print(f"[train] best weights copied to {MODEL_OUT}")
    metrics = YOLO(str(MODEL_OUT)).val(data=str(data), imgsz=imgsz, device=device, verbose=False)
    print(f"[train] val mAP50 {metrics.box.map50:.3f}  mAP50-95 {metrics.box.map:.3f}")
    print("        Judge the model on YOUR images (datasets/own valid split), not the public ones.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("step", choices=["download", "merge", "train", "all"])
    ap.add_argument("--model", default="yolov8n.pt", help="yolov8n.pt / yolov8s.pt / a previous best.pt")
    ap.add_argument("--epochs", type=int, default=80)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default=None, help="0 for the first GPU, cpu to force CPU")
    a = ap.parse_args()
    if a.step in ("download", "all"):
        download()
    if a.step in ("merge", "all"):
        merge()
    if a.step in ("train", "all"):
        train(a.model, a.epochs, a.imgsz, a.batch, a.device)


if __name__ == "__main__":
    main()
