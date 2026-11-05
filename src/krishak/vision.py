"""Plant proposals, row-based selection, and an optional learned classifier."""
from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

FEATURES = ("area_fraction", "aspect_ratio", "fill_ratio", "row_distance")


@dataclass
class Plant:
    center: tuple[float, float]
    box: tuple[int, int, int, int]
    features: np.ndarray
    weed: bool
    score: float | None


class PlantClassifier:
    """Standardized logistic regression; label 1 means weed, 0 means crop."""

    def __init__(self, mean, scale, weights):
        self.mean = np.asarray(mean, dtype=float)
        self.scale = np.asarray(scale, dtype=float)
        self.weights = np.asarray(weights, dtype=float)
        if (self.mean.shape != (4,) or self.scale.shape != (4,)
                or self.weights.shape != (5,) or np.any(self.scale <= 0)
                or not all(np.isfinite(a).all() for a in (self.mean, self.scale, self.weights))):
            raise ValueError("Invalid classifier parameters")

    @staticmethod
    def sigmoid(x):
        return 1 / (1 + np.exp(-np.clip(x, -40, 40)))

    @classmethod
    def fit(cls, x, y):
        x, y = np.asarray(x, float), np.asarray(y, float)
        if (x.ndim != 2 or x.shape[1] != 4 or y.shape != (len(x),)
                or set(y) != {0, 1} or not np.isfinite(x).all()):
            raise ValueError("Training needs finite four-feature rows and both labels 0 and 1")
        mean, scale = x.mean(axis=0), np.maximum(x.std(axis=0), 1e-6)
        design = np.column_stack(((x - mean) / scale, np.ones(len(x))))
        weights = np.zeros(5)
        for _ in range(2000):
            gradient = design.T @ (cls.sigmoid(design @ weights) - y) / len(y)
            gradient[:-1] += 0.01 * weights[:-1]
            weights -= 0.1 * gradient
        return cls(mean, scale, weights)

    def predict(self, x):
        return self.sigmoid(((np.asarray(x) - self.mean) / self.scale) @ self.weights[:-1]
                            + self.weights[-1])

    def save(self, path):
        Path(path).write_text(json.dumps({"features": FEATURES, "mean": self.mean.tolist(),
                                        "scale": self.scale.tolist(), "weights": self.weights.tolist()}, indent=2))

    @classmethod
    def load(cls, path):
        values = json.loads(Path(path).read_text())
        if values.pop("features") != list(FEATURES):
            raise ValueError("Model feature schema does not match")
        return cls(**values)


def detect(image, rows=(0.25, 0.75), crop_band=0.09, model=None):
    """Return plant proposals. Row coordinates and band width are image fractions.

    The baseline calls vegetation outside crop bands a weed candidate, not a
    botanical identification. Model scores are not calibrated probabilities.
    """
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("Expected a BGR image")
    if not rows or any(not 0 <= r <= 1 for r in rows) or not 0 < crop_band < 0.5:
        raise ValueError("Invalid crop-row geometry")
    height, width = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (30, 55, 35), (95, 255, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    plants = []
    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 30:
            continue
        moments = cv2.moments(contour)
        u, v = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
        x, y, w, h = cv2.boundingRect(contour)
        distance = min(abs(u / width - r) for r in rows)
        features = np.array([area / (height * width), w / h, area / (w * h), distance])
        score = float(model.predict(features)) if model else None
        weed = score >= 0.5 if model else distance > crop_band
        # Crop exclusion is enforced even when the classifier votes weed.
        plants.append(Plant((u, v), (x, y, w, h), features,
                            weed and distance > crop_band, score))
    return sorted(plants, key=lambda p: p.center), mask


def annotate(image, plants, rows=(0.25, 0.75), crop_band=0.09):
    result = image.copy()
    height, width = image.shape[:2]
    for row in rows:
        for edge in (row - crop_band, row + crop_band):
            cv2.line(result, (int(edge * width), 0), (int(edge * width), height), (200, 180, 60), 1)
    for plant in plants:
        x, y, w, h = plant.box
        color = (60, 70, 240) if plant.weed else (80, 210, 80)
        cv2.rectangle(result, (x, y), (x + w, y + h), color, 2)
        label = "weed candidate" if plant.weed else "retain"
        cv2.putText(result, label, (x, max(15, y - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    return result
