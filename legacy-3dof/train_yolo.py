"""
Train a YOLO model to recognise nuts, bolts and washers.

STEP 1 - Collect images (100-300 is enough for this simple scene)
    Use the SAME phone, mount height and white paper as the real setup.
    Vary: number of objects, positions, rotations, lighting, and mix all 3 classes.
    Save frames from the IP Webcam stream, e.g. one every second while you move parts.

STEP 2 - Label them (free tools: Roboflow, CVAT, Label Studio, labelImg)
    Draw a box around every object with classes:  nut, bolt, washer
    Export in "YOLOv8 / YOLO" format. You get a folder like:
        dataset/
          data.yaml
          train/images  train/labels
          valid/images  valid/labels

STEP 3 - Train (this script), then copy runs/detect/train/weights/best.pt
         next to app.py.  A free Google Colab GPU trains this in ~10 minutes.

    pip install ultralytics
    python train_yolo.py
"""
from ultralytics import YOLO

model = YOLO("yolov8n.pt")        # nano = fastest; use yolov8s.pt for more accuracy
model.train(
    data='/home/azad/CODE/projects/robot_arm_sorter/Nuts and Bolts.v1i.yolov8/data.yaml',     # path to the file exported from your labelling tool
    epochs=100,
    imgsz=640,
    batch=16,
    patience=20,                  # stop early if no improvement
    flipud=0.5, fliplr=0.5,       # top-down view: flipping both ways is valid
    degrees=180,                  # parts can lie at any rotation
)
print("Done. Copy runs/detect/train/weights/best.pt next to app.py")
