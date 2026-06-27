#!/usr/bin/env python
from __future__ import annotations
from typing import Callable, List, Optional
import numpy as np
try:
    import rospy

    def _log(level, msg):
        getattr(rospy, level)(msg)
except Exception:  
    def _log(level, msg):
        print(f"[{level}] {msg}")

class DetectionRegion(dict):
    def __init__(self, label: str, score: float,
                 mask: Optional[np.ndarray] = None,
                 bbox: Optional[tuple] = None):
        super().__init__(label=str(label), score=float(score), mask=mask, bbox=bbox)

    def centroid(self) -> tuple:
        mask = self.get("mask")
        if mask is not None and getattr(mask, "any", lambda: False)():
            ys, xs = np.where(mask)
            return int(xs.mean()), int(ys.mean())
        bbox = self.get("bbox")
        if bbox is not None:
            x1, y1, x2, y2 = bbox
            return int((x1 + x2) / 2), int((y1 + y2) / 2)
        return 0, 0

    def as_mask(self, height: int, width: int) -> np.ndarray:
        mask = self.get("mask")
        if mask is not None:
            return mask
        out = np.zeros((height, width), dtype=bool)
        bbox = self.get("bbox")
        if bbox is not None:
            x1, y1, x2, y2 = [int(v) for v in bbox]
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(width, x2), min(height, y2)
            out[y1:y2, x1:x2] = True
        return out

class OpenVocabBackend:
    name = "base"

    def detect(self, rgb_image: np.ndarray, queries: List[str]) -> List[DetectionRegion]:
        raise NotImplementedError

class YoloWorldBackend(OpenVocabBackend):
    name = "yolo_world"
    def __init__(self, weights: str, device: str = "cpu", conf: float = 0.2):
        self.weights = weights
        self.device = device
        self.conf = conf
        self._model = None
        self._classes_cache = None

    def _ensure_model(self):
        if self._model is None:
            from ultralytics import YOLOWorld  # lazy import
            self._model = YOLOWorld(self.weights)
            _log("loginfo", f"[Perception] Loaded YOLO-World weights: {self.weights}")

    def detect(self, rgb_image, queries):
        self._ensure_model()
        if not queries:
            return []
        if queries != self._classes_cache:
            self._model.set_classes(list(queries))
            self._classes_cache = list(queries)
        results = self._model.predict(
            rgb_image, conf=self.conf, device=self.device, verbose=False
        )
        regions: List[DetectionRegion] = []
        if not results:
            return regions
        res = results[0]
        names = getattr(res, "names", None) or getattr(self._model, "names", {})
        boxes = getattr(res, "boxes", None)
        if boxes is None:
            return regions
        for box in boxes:
            cls_id = int(box.cls.item())
            score = float(box.conf.item())
            label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
            xyxy = box.xyxy[0].tolist()
            regions.append(DetectionRegion(label=label, score=score,
                                           bbox=tuple(int(v) for v in xyxy)))
        return regions

class YoloEBackend(OpenVocabBackend):
    name = "yoloe"

    def __init__(self, weights: str, device: str = "cpu", conf: float = 0.2):
        self.weights = weights
        self.device = device
        self.conf = conf
        self._model = None
        self._classes_cache = None

    def _ensure_model(self):
        if self._model is None:
            from ultralytics import YOLO  
            self._model = YOLO(self.weights)
            _log("loginfo", f"[Perception] Loaded YOLOE weights: {self.weights}")

    def detect(self, rgb_image, queries):
        self._ensure_model()
        if not queries:
            return []
        if queries != self._classes_cache:
            self._model.set_classes(list(queries), self._model.get_text_pe(list(queries)))
            self._classes_cache = list(queries)
        results = self._model.predict(
            rgb_image, conf=self.conf, device=self.device, verbose=False
        )
        regions: List[DetectionRegion] = []
        if not results:
            return regions
        res = results[0]
        names = getattr(res, "names", None) or {}
        boxes = getattr(res, "boxes", None)
        masks = getattr(res, "masks", None)
        if boxes is None:
            return regions
        mask_data = masks.data.cpu().numpy().astype(bool) if masks is not None else None
        for i, box in enumerate(boxes):
            cls_id = int(box.cls.item())
            score = float(box.conf.item())
            label = names.get(cls_id, str(cls_id)) if isinstance(names, dict) else str(cls_id)
            mask = mask_data[i] if mask_data is not None and i < len(mask_data) else None
            xyxy = tuple(int(v) for v in box.xyxy[0].tolist())
            regions.append(DetectionRegion(label=label, score=score, mask=mask, bbox=xyxy))
        return regions


class Sam3Backend(OpenVocabBackend):
    name = "sam3"
    def __init__(self, weights: str, device: str = "cpu", conf: float = 0.2):
        self.weights = weights
        self.device = device
        self.conf = conf
        self._model = None

    def _ensure_model(self):
        if self._model is None:
            from ultralytics import SAM 
            self._model = SAM(self.weights)
            _log("loginfo", f"[Perception] Loaded SAM 3 weights: {self.weights}")

    def _predict(self, rgb_image, prompt):
        for kwargs in ({"prompts": [prompt]}, {"texts": [prompt]}, {"labels": [prompt]}):
            try:
                return self._model(rgb_image, device=self.device, verbose=False, **kwargs)
            except TypeError:
                continue
        return self._model(rgb_image, prompt, device=self.device, verbose=False)

    def detect(self, rgb_image, queries):
        self._ensure_model()
        regions: List[DetectionRegion] = []
        for phrase in queries:
            try:
                results = self._predict(rgb_image, phrase)
            except Exception as e:
                _log("logwarn",
                     f"[Perception] SAM 3 inference failed for '{phrase}': {e}. "
                     f"Verify the text-prompt API for your installed sam3/ultralytics "
                     f"version (see github.com/facebookresearch/sam3).")
                continue
            if not results:
                continue
            res = results[0]
            masks = getattr(res, "masks", None)
            if masks is None:
                continue
            mask_data = masks.data.cpu().numpy().astype(bool)
            boxes = getattr(res, "boxes", None)
            for i in range(len(mask_data)):
                score = float(boxes[i].conf.item()) if boxes is not None and i < len(boxes) else 1.0
                if score < self.conf:
                    continue
                regions.append(DetectionRegion(label=phrase, score=score, mask=mask_data[i]))
        return regions

_BACKENDS = {
    "yolo_world": YoloWorldBackend,
    "yoloe": YoloEBackend,
    "sam3": Sam3Backend,
}

def create_backend(name: str, get_param: Callable, device: str = "cpu") -> Optional[OpenVocabBackend]:
    name = (name or "").lower()
    if name not in _BACKENDS:
        return None
    weights = get_param(f"perception/open_vocab/{name}_weights", "")
    conf = float(get_param("perception/detection_confidence_threshold", 0.2))
    if not weights:
        _log("logwarn",
             f"[Perception] No weights configured for backend '{name}' "
             f"(perception/open_vocab/{name}_weights). Falling back to clip_sam.")
        return None
    return _BACKENDS[name](weights=weights, device=device, conf=conf)
