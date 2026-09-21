"""Pure CVisionLib image primitives; never acquires frames or changes hardware.

Authority: SSD decompiled_src_vision/CVisionLib/ClassFrameGrabber.cs.
The OEM uses OpenCV 3.2; the Python dependency must be qualified separately.
Encoded inputs are existing camera snapshot content, NOT filenames.  Arrays are
uint8 BGR camera pixels (or explicitly preconverted 2-D grayscale).  No resize,
normalization, inferred confidence, or calibration substitution is performed.
"""
from __future__ import annotations

from typing import NamedTuple

import cv2
import numpy as np


class PatternMatch(NamedTuple):
    maximum: float
    x: int
    y: int


def _pixels(image: bytes | np.ndarray, *, grayscale: bool = False) -> np.ndarray:
    if isinstance(image, bytes):
        if not image:
            raise ValueError("empty encoded image")
        value = cv2.imdecode(np.frombuffer(image, np.uint8), 0 if grayscale else 1)
        if value is None:
            raise ValueError("image cannot be decoded")
    elif isinstance(image, np.ndarray):
        value = image
    else:
        raise TypeError("image must be encoded bytes or uint8 pixel array")
    if value.dtype != np.uint8 or value.size == 0:
        raise ValueError("image must contain uint8 pixels")
    if value.ndim != 2 and not (value.ndim == 3 and value.shape[2] == 3):
        raise ValueError("expected grayscale or three-channel BGR image")
    if grayscale and value.ndim == 3:
        return cv2.cvtColor(value, 6)
    return value


def _gray(image: bytes | np.ndarray, *, file_mode: bool = False, code: int = 7) -> np.ndarray:
    value = _pixels(image, grayscale=file_mode)
    # Numeric 7 is deliberate: CVision's live branch uses RGB2GRAY on the
    # captured channel order. Do not silently replace it with BGR2GRAY (6).
    return cv2.cvtColor(value, code) if value.ndim == 3 else value


def _roi(gray: np.ndarray, x: int, y: int, width: int, height: int) -> np.ndarray:
    if x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > gray.shape[1] or y + height > gray.shape[0]:
        raise ValueError("image does not contain the OEM inspection ROI")
    return gray[y:y + height, x:x + width]


def _hist(gray: np.ndarray) -> np.ndarray:
    # OEM exclusive upper limit is 255, NOT 256. White pixels are excluded.
    return cv2.calcHist([gray], [0], None, [256], [0, 255]).ravel()


def match_pattern(image: bytes | np.ndarray, template: bytes | np.ndarray,
                  match_method: int = 5, *, file_mode: bool = False) -> PatternMatch:
    """matchPattern lines 12605-12956: always MAX, including SQDIFF methods.

    file_mode=True reproduces fname!=empty (imread grayscale). Default reproduces
    already-acquired live pixels. Templates always use imread grayscale semantics.
    """
    if isinstance(match_method, bool) or match_method not in range(6):
        raise ValueError("OpenCV template method must be 0..5")
    gray = _gray(image, file_mode=file_mode)
    pattern = _pixels(template, grayscale=True)
    if pattern.shape[0] > gray.shape[0] or pattern.shape[1] > gray.shape[1]:
        raise ValueError("template exceeds image dimensions")
    result = cv2.matchTemplate(gray, pattern, match_method)
    _, maximum, _, location = cv2.minMaxLoc(result)
    return PatternMatch(float(maximum), int(location[0]), int(location[1]))


def check_purification_station(image: bytes | np.ndarray) -> bool:
    """7428-7688: >20000 pixels strictly >40, full-width TOP half."""
    gray = _gray(image)
    return int(np.count_nonzero(gray[:gray.shape[0] // 2, :] > 40)) > 20000


def check_output_plate(image: bytes | np.ndarray, threshold: int, pixelcount: int) -> bool:
    """7690-7950: BOTTOM LEFT quadrant, strict intensity, inclusive count."""
    gray = _gray(image)
    return int(np.count_nonzero(gray[gray.shape[0] // 2:, :gray.shape[1] // 2] > threshold)) >= pixelcount


def check_trough(image: bytes | np.ndarray, threshold: int) -> bool:
    """9014-9282: right half, 190 rows beginning at mid-height; >=2000."""
    gray = _gray(image)
    h, w = gray.shape
    roi = _roi(gray, w // 2, h // 2, w - w // 2, 190)
    return int(np.count_nonzero(roi > threshold)) >= 2000


def check_notch(image: bytes | np.ndarray, *, file_mode: bool = False) -> int:
    """3515-3790: CVL_Status 0/1/2; notch uniquely uses live color code 6."""
    gray = _gray(image, file_mode=file_mode, code=6)
    means = [cv2.mean(_roi(gray, x, y, 60, 60))[0]
             for x, y in ((290, 220), (200, 220), (290, 130))]
    # Preserve native IEEE double division, including 0/0 NaN and nonzero/0 inf.
    with np.errstate(divide="ignore", invalid="ignore"):
        first = np.float64(abs(means[0] - means[1])) / means[1]
        second = np.float64(abs(means[1] - means[2])) / means[2]
    return 1 if not second > 2.0 else (2 if first < 0.2 else 0)


def locate_cover(image: bytes | np.ndarray, *, file_mode: bool = False) -> bool:
    """5376-5726: median5 then original histogram extrema predicate."""
    histogram = _hist(cv2.medianBlur(_gray(image, file_mode=file_mode), 5))
    maximum = np.float32(0)
    highest = lowest = 255
    for index in range(255, -1, -1):
        value = histogram[index]
        if value > maximum:
            maximum = value
        if value > 10 and highest == 255:
            highest = index
        if highest != 255 and value > 10:
            lowest = index
    total = np.float32(0)
    for index in range(15, 35):
        total = np.float32(total + histogram[index])
    return bool(lowest > 35 or (total < maximum and highest > 100))


def check_biosecurity_cover(image: bytes | np.ndarray, *, file_mode: bool = False) -> bool:
    """11918-12271: full histogram valley, then top 300-row histogram."""
    gray = _gray(image, file_mode=file_mode)
    roi = _roi(gray, 0, 0, gray.shape[1], 300)
    histogram = _hist(gray)
    valley = int(np.argmin(histogram[50:150])) + 50
    dark = np.float32(0)
    for index in range(25):
        dark = np.float32(dark + histogram[index])
    if histogram[valley] >= 350 and dark <= 90000:
        return False
    top = _hist(roi)
    count = np.float32(0)
    for index in range(valley, 255):
        count = np.float32(count + top[index])
    return bool(count > 10000)


def label_dark_frame_valid(image: bytes | np.ndarray) -> bool:
    """checkLabel first stage: histogram max-min >=2000 (not intensity range)."""
    minimum, maximum, _, _ = cv2.minMaxLoc(_hist(_gray(image)))
    return maximum - minimum >= 2000.0


def check_label(led2_off: bytes | np.ndarray, led2_on: bytes | np.ndarray) -> bool:
    """9284-9859 computational CheckCamera primitive, TWO real exposures.

    Acquisition owner must follow OEM: LED2 off -> first frame; if histogram
    passes, LED2 on -> sleep 1000ms -> second frame. No illumination mutation
    happens here. Same-frame reuse must not be substituted for this sequence.
    """
    first = _gray(led2_off)
    minimum, maximum, _, _ = cv2.minMaxLoc(_hist(first))
    if maximum - minimum < 2000.0:
        return False
    second = _gray(led2_on)
    if first.shape != second.shape:
        raise ValueError("checkLabel requires matching frame dimensions")
    _, bright_maximum, _, _ = cv2.minMaxLoc(second)
    _, correlation, _, _ = cv2.minMaxLoc(cv2.matchTemplate(first, second, 5))
    return bool(bright_maximum >= 50.0 and abs(correlation) <= 0.9)


def output_location_from_scores(cover: float, output: float, foil: float, empty: float) -> int:
    """ControlLib 3802: keep OEM precedence; plain output has no .7 gate."""
    if not all(np.isfinite(v) for v in (cover, output, foil, empty)):
        raise ValueError("nonfinite pattern score")
    maximum = max(cover, output, foil, empty)
    if maximum == cover and maximum > 0.7:
        return 1
    return 2 if maximum == output or (maximum == foil and maximum > 0.7) else 0


def reagent_cover_from_scores(cover: float, reagent: float, empty: float) -> bool:
    """ControlLib 3836-3840: cover wins ties, no confidence cutoff."""
    if not all(np.isfinite(v) for v in (cover, reagent, empty)):
        raise ValueError("nonfinite pattern score")
    return max(cover, reagent, empty) == cover
