"""Pure OEM strip computations; no acquisition, motion or inferred inventory.

ClassFrameGrabber.cs: inspectStrip 9869–10774, findStripHandle 11396–11915.
OpenCV 3.2 source; Python operations qualified separately on 3.4.18.
"""
from __future__ import annotations

import math

import cv2
import numpy as np

from .oem_inspection import _gray, _hist, _roi
from .oem_tips import get_blobs


def _handle_threshold(histogram: np.ndarray) -> int:
    peak = 0.0
    valley = float(np.finfo(np.float32).max)
    selected = 0
    finding_peak = True
    finding_valley = False
    # Deliberately skips bins 252..255 and keeps first (highest) tied valley.
    for index in range(251, -1, -1):
        count = float(histogram[index])
        if finding_peak:
            if count > peak:
                peak = count
            elif count < peak * 0.5 and peak > 250.0:
                finding_peak = False
                finding_valley = True
        if finding_valley:
            if count < valley:
                valley, selected = count, index
            elif count > valley * 25.0:
                break
    return selected if selected >= 100 else 180


def find_strip_handle(image: bytes | np.ndarray, adjustment: int,
                      equalize: bool, *, file_mode: bool = False) -> tuple[int, int, int]:
    """Return (area, ROI-local centroid x, y); no retained blob gives (0,0,0).

    adjustment translates ROI x, not its width. Live RGB2GRAY numeric7 and
    file grayscale decoding deliberately differ. Largest area wins; equal
    areas retain the first row-major component. No added area acceptance gate.
    """
    if isinstance(adjustment, bool) or not isinstance(adjustment, int):
        raise ValueError("adjustment must be an integer")
    gray = _gray(image, file_mode=file_mode)
    if equalize:
        gray = cv2.equalizeHist(gray)
    crop = cv2.medianBlur(_roi(gray, 210 + adjustment, 110, 220, 260), 5)
    threshold = _handle_threshold(_hist(crop))
    best = (0, 0, 0)
    for blob in get_blobs(cv2.inRange(crop, 0, threshold)):
        if blob.area > best[0]:
            best = (blob.area, blob.x, blob.y)
    return best


def _strip_line_bits(lines: np.ndarray | None) -> int:
    bits = 0
    if lines is None:
        return bits
    for x1, y1, x2, y2 in lines.reshape(-1, 4):
        x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
        if x1 == 549 or x2 == 549:
            continue
        # Source casts both operands, then Atan2's double result, to float32.
        angle = float(np.float32(math.atan2(float(np.float32(y2 - y1)),
                                            float(np.float32(x2 - x1)))))
        if not any(abs(angle - target) < 0.174 for target in (1.57, 4.712, -1.57, -4.712)):
            continue
        for center, bit in ((56, 16), (186, 8), (316, 4), (446, 2), (576, 1)):
            if abs(x1 - center) < 30:
                bits |= bit
                break
    return bits


def _strip_morphology(gray: np.ndarray, y: int) -> np.ndarray:
    """Preserve cv::Mat ROI parent reads AND writes in D,D,E,E,E,E.

    NumPy slices lose the C++ parent's locateROI metadata at the Python cv2
    boundary. Passes 1,3,5 therefore operate on the full parent and crop their
    output; passes 2,4,6 read the independent scratch and write into the ROI.
    Do not replace this with dilate(iterations=2)/erode(iterations=4).
    """
    roi = _roi(gray, 0, y, gray.shape[1], 100)
    for operation in (cv2.dilate, cv2.erode, cv2.erode):
        scratch = operation(gray, None, anchor=(-1, -1), iterations=1,
                            borderType=cv2.BORDER_CONSTANT)[y:y + 100, :].copy()
        roi[:] = operation(scratch, None, anchor=(-1, -1), iterations=1,
                           borderType=cv2.BORDER_CONSTANT)
    return roi


def _source_int(value: float) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("Canny threshold must be a finite int32-compatible number")
    result = int(value)
    if not -(1 << 31) <= result < (1 << 31):
        raise ValueError("Canny threshold exceeds source int32")
    return result


def inspect_strip(image: bytes | np.ndarray, threLow: float, threHigh: float,
                  equalize: bool, *, file_mode: bool = False) -> int:
    """Four packed edge masks (top band highest byte), NOT strip colors.

    Source ControlLib casts thresholds to int; no custom positivity/order gate.
    Color conversion is BGR2GRAY numeric6 in BOTH live and file branches. File
    mode reproduces imread(-1) then conversion, not decoder grayscale rounding.
    Explicit 2-D arrays are already-gray computational inputs only.
    """
    low, high = _source_int(threLow), _source_int(threHigh)
    if file_mode and isinstance(image, bytes):
        if not image:
            raise ValueError("empty encoded image")
        pixels = cv2.imdecode(np.frombuffer(image, np.uint8), -1)
        if pixels is None or pixels.dtype != np.uint8:
            raise ValueError("expected decodable uint8 color image")
        gray = cv2.cvtColor(pixels, 6)
    else:
        gray = _gray(image, code=6)
    gray = cv2.equalizeHist(gray) if equalize else gray.copy()
    result = 0
    for y in (1, 127, 253, 379):
        band = _strip_morphology(gray, y)
        edges = cv2.Canny(band, low, high, apertureSize=3, L2gradient=False)
        lines = cv2.HoughLinesP(edges, 1.0, math.pi / 180.0, 30,
                                minLineLength=20.0, maxLineGap=10.0)
        result = (result << 8) | _strip_line_bits(lines)
    return result
