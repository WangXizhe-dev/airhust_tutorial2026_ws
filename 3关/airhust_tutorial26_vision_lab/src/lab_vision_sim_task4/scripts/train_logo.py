"""Generate labelled Gazebo-like pads and fine-tune the supplied nano detector."""
import argparse
import shutil
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def make_split(texture, root, split, count, seed):
    rng = np.random.default_rng(seed)
    for kind in ("images", "labels"):
        (root / kind / split).mkdir(parents=True, exist_ok=True)
    for i in range(count):
        side = 224
        level = rng.uniform(65, 190)
        scene = np.clip(rng.normal(level, 4, (side, side, 3)), 0, 255).astype("uint8")
        label = ""
        if i % 8:
            size = rng.uniform(30, 120)
            x, y = rng.uniform(size * .72, side - size * .72, 2)
            quad = cv2.boxPoints(((x, y), (size, size), rng.uniform(-180, 180)))
            quad += rng.uniform(-0.04, 0.04, (4, 2)).astype("float32") * size
            h, w = texture.shape[:2]
            src = np.float32([[0,h-1],[0,0],[w-1,0],[w-1,h-1]])
            transform = cv2.getPerspectiveTransform(src, quad)
            patch = cv2.convertScaleAbs(texture, alpha=rng.uniform(.55,1.05), beta=rng.uniform(0,12))
            cv2.warpPerspective(patch, transform, (side,side), dst=scene, borderMode=cv2.BORDER_TRANSPARENT)
            lo = quad.min(axis=0).clip(0,side)
            hi = quad.max(axis=0).clip(0,side)
            values = np.r_[(hi+lo)/2,hi-lo] / side
            label = "0 " + " ".join(f"{v:.6f}" for v in values) + "\n"
        cv2.imwrite(str(root/"images"/split/f"{i}.jpg"), scene)
        (root/"labels"/split/f"{i}.txt").write_text(label)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("work", type=Path)
    root = parser.parse_args().work.resolve()
    pkg = Path(__file__).resolve().parents[1]
    texture = cv2.imread(str(pkg/"models/logo_pad/materials/textures/logo.png"))
    make_split(texture, root, "train", 80, 42)
    make_split(texture, root, "val", 24, 314)
    cfg = root/"data.yaml"
    cfg.write_text(f"path: {root}\ntrain: images/train\nval: images/val\nnames: [AirHust]\n")
    base = pkg.parent/"lab_vision_sim_task3/models/best.pt"
    model = YOLO(str(base))
    model.train(data=str(cfg), epochs=14, imgsz=224, batch=8, workers=0, device="cpu",
                optimizer="AdamW", lr0=.001, mosaic=0, plots=False, amp=False,
                project=str(root/"runs"), name="logo", seed=42)
    shutil.copy2(model.trainer.best, pkg/"models/logo.pt")
