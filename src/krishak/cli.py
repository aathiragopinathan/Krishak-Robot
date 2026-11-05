"""CLI entry points for the image demo and supervised plant classifier."""
import argparse
import csv
import json
from pathlib import Path

import cv2
import numpy as np

from .controller import Controller, State
from .robotics import Arm, GroundMapper, SeedArm, SimRobot
from .vision import FEATURES, PlantClassifier, annotate, detect


def synthetic_field():
    rng = np.random.default_rng(7)
    image = np.full((480, 640, 3), (45, 65, 85), dtype=np.uint8)
    noise = rng.integers(-10, 11, image.shape, dtype=np.int16)
    image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    for x in (160, 480):
        for y in (70, 180, 290, 400):
            cv2.ellipse(image, (x, y), (15, 26), 15, 0, 360, (35, 145, 45), -1)
    for x, y in ((295, 120), (355, 270), (300, 385)):
        for angle in (0, 60, 120):
            cv2.ellipse(image, (x, y), (18, 7), angle, 0, 360, (40, 170, 65), -1)
    return image


def read_labels(path):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    x = np.array([[float(row[key]) for key in FEATURES] for row in rows])
    y = np.array([float(row["label"]) for row in rows])
    if len(rows) == 0 or not np.isfinite(x).all() or not set(y).issubset({0, 1}):
        raise ValueError("CSV needs finite features and binary labels")
    return x, y


def run_demo(args):
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    image = cv2.imread(args.image) if args.image else synthetic_field()
    if image is None:
        raise ValueError(f"Could not read image: {args.image}")
    model = PlantClassifier.load(args.model) if args.model else None
    plants, mask = detect(image, args.rows, args.crop_band, model)
    for name, pixels in (("input.png", image), ("mask.png", mask),
                         ("detections.png", annotate(image, plants, args.rows, args.crop_band))):
        if not cv2.imwrite(str(output / name), pixels):
            raise OSError(f"Could not write {name}")
    with (output / "features.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["id", "u", "v", *FEATURES, "label"])
        for index, plant in enumerate(plants):
            writer.writerow([index, *plant.center, *plant.features, ""])
    robot = SimRobot(arm=SeedArm() if args.arm == "seed" else Arm())
    controller = Controller(robot)
    # External images require supplied calibration before simulated targeting.
    calibration = json.loads(Path(args.calibration).read_text()) if args.calibration else None
    if not args.image and calibration is None:
        calibration = [[0, -0.00035, 0.37], [0.0005, 0, -0.16], [0, 0, 1]]
    if calibration is not None:
        mapper = GroundMapper(calibration)
        controller.begin([mapper.project(p.center) for p in plants if p.weed])
        for _ in range(1000):
            controller.tick()
            if controller.state in (State.DONE, State.FAULT):
                break
        else:
            controller.tick(emergency_stop=True)
    report = {"input": args.image or "synthetic field (seed 7)",
              "arm_model": args.arm,
              "detector": "logistic classifier + crop exclusion" if model else "HSV + crop-row baseline",
              "plants": len(plants), "weed_candidates": sum(p.weed for p in plants),
              "state": controller.state.value, "error": controller.error,
              "actions": robot.actions, "trace": controller.trace,
              "note": "Simulation only. External images without calibration run perception only."}
    (output / "run.json").write_text(json.dumps(report, indent=2))
    print(json.dumps({k: report[k] for k in ("plants", "weed_candidates", "state", "error")}, indent=2))
    print(f"Artifacts: {output.resolve()}")
    return 1 if controller.state == State.FAULT else 0


def main():
    parser = argparse.ArgumentParser(description="Krishak vision and mobile-manipulation prototype")
    sub = parser.add_subparsers(dest="command", required=True)
    demo = sub.add_parser("demo", help="Process an image and run the simulated task loop")
    demo.add_argument("--image")
    demo.add_argument("--output", default="outputs/demo")
    demo.add_argument("--model")
    demo.add_argument("--arm", choices=["six-dof", "seed"], default="six-dof",
                      help="Example six-joint chain or paper-inspired three-joint positioning arm with gripper")
    demo.add_argument("--rows", nargs="+", type=float, default=[0.25, 0.75])
    demo.add_argument("--crop-band", type=float, default=0.09)
    demo.add_argument("--calibration", help="JSON 3x3 pixel-to-arm-ground homography")
    train = sub.add_parser("train", help="Fit logistic regression on labeled plant features")
    train.add_argument("csv")
    train.add_argument("model")
    evaluate = sub.add_parser("evaluate", help="Evaluate a saved classifier on held-out plants")
    evaluate.add_argument("csv")
    evaluate.add_argument("model")
    args = parser.parse_args()
    try:
        if args.command == "demo":
            return run_demo(args)
        x, y = read_labels(args.csv)
        if args.command == "train":
            PlantClassifier.fit(x, y).save(args.model)
            print(f"Saved model to {args.model}; evaluate on a separate capture session.")
        else:
            prediction = PlantClassifier.load(args.model).predict(x) >= 0.5
            tp = int(np.sum(prediction & (y == 1)))
            fp = int(np.sum(prediction & (y == 0)))
            fn = int(np.sum(~prediction & (y == 1)))
            tn = int(np.sum(~prediction & (y == 0)))
            print(json.dumps({"samples": len(y), "tp": tp, "fp": fp, "fn": fn, "tn": tn,
                              "accuracy": float(np.mean(prediction == y)),
                              "precision": tp / (tp + fp) if tp + fp else None,
                              "recall": tp / (tp + fn) if tp + fn else None}, indent=2))
        return 0
    except (ValueError, OSError, KeyError) as exc:
        parser.exit(2, f"error: {exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
