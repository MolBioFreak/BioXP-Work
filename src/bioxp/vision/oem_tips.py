"""CVision GetBlobs/missingTps and ControlLib.setTipInfo, without hardware.

Source: ClassFrameGrabber.cs 1790-2476 and 14349-14519;
ControlLib.cs 9023-9086, CommonLib/wellID.cs (row-major A1..H12).
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

from .oem_inspection import _gray, _hist, _roi


@dataclass(frozen=True)
class Blob:
    xmin: int
    xmax: int
    ymin: int
    ymax: int
    x: int
    y: int
    area: int


def get_blobs(binary: np.ndarray) -> tuple[Blob, ...]:
    """Literal OEM 8-neighbor BFS, interior seeds, inclusive extrema.

    Keep components >=500 pixels which touch neither left nor right border.
    Top/bottom border contact is allowed. Centroids truncate integer division.
    Not contour area, fitted ellipses, or a different segmentation algorithm.
    """
    if not isinstance(binary, np.ndarray) or binary.dtype != np.uint8 or binary.ndim != 2:
        raise ValueError("GetBlobs requires a uint8 single-channel mask")
    height, width = binary.shape
    seen = np.zeros(binary.shape, dtype=bool)
    blobs = []
    for y in range(1, height - 1):
        for x in range(1, width - 1):
            if not binary[y, x] or seen[y, x]:
                continue
            queue = deque([(x, y)])
            seen[y, x] = True
            count = sx = sy = 0
            xmin, ymin, xmax, ymax = width + 1, height + 1, 0, 0
            while queue:
                px, py = queue.popleft()
                count += 1
                sx += px
                sy += py
                xmin, xmax = min(xmin, px), max(xmax, px)
                ymin, ymax = min(ymin, py), max(ymax, py)
                for ny in range(max(0, py - 1), min(height, py + 2)):
                    for nx in range(max(0, px - 1), min(width, px + 2)):
                        if binary[ny, nx] and not seen[ny, nx]:
                            seen[ny, nx] = True
                            queue.append((nx, ny))
            if count >= 500 and xmin != 0 and xmax != width - 1:
                blobs.append(Blob(xmin, xmax, ymin, ymax, sx // count, sy // count, count))
    return tuple(blobs)


TIP_CENTERS = tuple((x, y) for y in (51, 177, 303, 429) for x in (111, 237, 363, 489))


def missing_tips(image: bytes | np.ndarray, threshold: int, tiptype: int,
                 *, file_mode: bool = False) -> int:
    """OEM 4-argument missingTps: encoded missing-hole bitmask <<8 or -1.

    50/200 are actual OEM tiptype values, not tray capacity. The 50 branch's
    >=16 blob-vector gate includes its original one value-initialized element;
    this is >=15 real retained blobs, NOT an invented >=16-hole gate.
    A zero return is computed absence of qualifying holes, not inventory proof.
    """
    if tiptype not in (50, 200):
        raise ValueError("only source tip types 50 and 200 are supported")
    if isinstance(threshold, bool) or not isinstance(threshold, int):
        raise ValueError("threshold must be an integer")
    if tiptype == 50 and not 10 <= threshold <= 246:
        raise ValueError("OEM threshold +/-10 must address histogram bins 0..255")
    gray = _gray(image, file_mode=file_mode)
    cropped = _roi(gray, 20, 0, gray.shape[1] - 40, gray.shape[0])
    histogram = _hist(cropped)
    if tiptype == 50:
        selected = threshold - 10 + int(np.argmin(histogram[threshold - 10:threshold + 10]))
    else:
        # Reverse scan uses strict comparisons: ties retain highest index.
        peak = 0
        peak_count = np.float32(0)
        selected = 0
        valley_count = np.float32(np.finfo(np.float32).max)
        dark = np.float32(0)
        for index in range(255, -1, -1):
            value = histogram[index]
            if index > 10 and peak_count < value:
                peak_count, peak = value, index
            elif index < 5:
                dark = np.float32(dark + value)
            if 51 <= index <= 149 and valley_count > value:
                valley_count, selected = value, index
        if dark < 80000 and peak < 30:
            return -1
    blobs = get_blobs(cv2.inRange(cropped, 0, selected))
    if tiptype == 50 and len(blobs) + 1 < 16:
        return -1
    # Source calls GetBlobs on the empty mat4 after the 50 branch. That appends
    # nothing to the existing vector; do not discard the first pass's blobs.
    result = 0
    for blob in blobs:
        if not 4001 <= blob.area <= 9499:
            continue
        if tiptype == 200 and not (86 <= blob.xmax - blob.xmin <= 111 and 86 <= blob.ymax - blob.ymin <= 111):
            continue
        distances = [np.sqrt(np.float32(np.float32((x - blob.x) ** 2) + np.float32((y - blob.y) ** 2)))
                     for x, y in TIP_CENTERS]
        index = int(np.argmin(distances))  # first minimum wins, as in OEM
        result |= 1 << index
    return result << 8


def missing_tip_wells(tipstatus: int, region: int) -> tuple[int, ...]:
    """Pure setTipInfo mapping: bit0 D1, bit1 C1,... region0; IDs 0..95.

    Regions traverse A1,A5,A9,E9,E5,E1. Caller must not pass -1 (tray failure).
    This returns only observed missing wells; never fabricates present inventory.
    """
    if isinstance(region, bool) or region not in range(6):
        raise ValueError("tip region must be 0..5")
    if not isinstance(tipstatus, int) or tipstatus < 0 or tipstatus & ~0xFFFF00:
        raise ValueError("tipstatus must be a valid OEM 16-bit missing mask shifted by 8")
    origin = (0, 4, 8, 56, 52, 48)[region]
    row, column = divmod(origin, 12)
    bits = tipstatus >> 8
    return tuple((row + 3 - j) * 12 + column + i
                 for i in range(4) for j in range(4) if bits & (1 << (i * 4 + j)))
