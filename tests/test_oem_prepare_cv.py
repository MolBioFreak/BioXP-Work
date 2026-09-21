"""Computational CV acceptance with immutable OEM templates and saved frames.

Run only under the external device/network-isolated frozen runner. Fixture
images remain outside the repository; missing fixtures fail rather than skip.
"""
from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import pytest

from bioxp.vision import oem_inspection as vision
from bioxp.vision import oem_tips as tips


EVIDENCE = Path(os.environ['OEM_CV_EVIDENCE'])
FIXTURES = EVIDENCE / 'fixtures'
MANIFEST = json.loads((EVIDENCE / 'fixture-manifest.json').read_text())
TEMPLATES = sorted(name for name in MANIFEST if name.startswith('templates/'))
SNAPSHOTS = sorted(name for name in MANIFEST if name.startswith('snapshots/'))


def fixture_bytes(name):
    data = (FIXTURES / name).read_bytes()
    assert hashlib.sha256(data).hexdigest() == MANIFEST[name]['sha256']
    return data


def record(name, value):
    target = Path(os.environ['OEM_CV_RESULTS'])
    target.mkdir(exist_ok=True)
    (target / (name + '.json')).write_text(json.dumps(value, indent=2) + '\n')


def configured(item, parameter):
    root = ET.fromstring(fixture_bytes('configuration/InspectionSettings.xml'))
    rows = root.find('{*}Settings3200')
    for row in rows:
        if row.find('{*}Key').text == item:
            values = row.find('{*}Value').find('{*}Parameters')
            for value in values:
                if value.find('{*}Key').text == parameter:
                    return int(value.find('{*}Value').text)
    raise AssertionError((item, parameter))


@pytest.mark.parametrize('name', TEMPLATES)
def test_oem_template_true_pixels_translation_and_negative_polarity(name):
    raw = fixture_bytes(name)
    gray = cv2.imdecode(np.frombuffer(raw, np.uint8), 0)
    positive = vision.match_pattern(raw, raw, file_mode=True)
    negative = vision.match_pattern(255 - gray, raw)
    shifted = np.pad(gray, ((13, 17), (19, 23)))
    translated = vision.match_pattern(shifted, raw)
    assert positive.maximum > .999
    assert positive.x == positive.y == 0
    assert negative.maximum < -.999
    assert translated.maximum > .999 and translated[1:] == (19, 13)
    record(Path(name).stem, {'fixture': name, 'shape': gray.shape,
           'self_match': positive, 'inverted_match': negative, 'translated_match': translated,
           'transformations': 'grayscale decode; intensity inversion; exact pixels padded x19/y13'})


@pytest.mark.parametrize('name', SNAPSHOTS)
def test_real_oem_snapshots_pixel_predicates_against_source_loops(name):
    raw = fixture_bytes(name)
    bgr = cv2.imdecode(np.frombuffer(raw, np.uint8), 1)
    gray = cv2.cvtColor(bgr, 7)
    h, w = gray.shape
    output_threshold = configured('OutputPlateInspection', 'threshold')
    output_count = configured('OutputPlateInspection', 'pixelCount')
    trough_threshold = configured('TroughInspection', 'threshold')
    # Independent literal scalar loops, not the production ROI/count helper.
    p = sum(int(gray[y, x]) > 40 for x in range(w) for y in range(h // 2))
    o = sum(int(gray[y, x]) > output_threshold for x in range(w // 2) for y in range(h // 2, h))
    t = sum(int(gray[y, x]) > trough_threshold for x in range(w // 2, w) for y in range(h // 2, h // 2 + 190))
    assert vision.check_purification_station(raw) == (p > 20000)
    assert vision.check_output_plate(raw, output_threshold, output_count) == (o >= output_count)
    assert vision.check_trough(raw, trough_threshold) == (t >= 2000)
    report = {'fixture': name, 'shape': bgr.shape, 'top_bright_count': p,
              'output_bright_count': o, 'trough_bright_count': t,
              'locate_cover': vision.locate_cover(raw, file_mode=True),
              'notch_status': vision.check_notch(raw, file_mode=True),
              'biosecurity_cover': vision.check_biosecurity_cover(raw, file_mode=True),
              'qualification': 'computed saved-frame values; filename labels are not ground-truth inventory'}
    if 'tip-tray' in name:
        report['missing_50'] = tips.missing_tips(raw, configured('TipInspection', 'threshold'), 50, file_mode=True)
        report['missing_200'] = tips.missing_tips(raw, configured('TipInspection', 'threshold'), 200, file_mode=True)
    record('frame-' + Path(name).stem, report)


def test_existing_immutable_bundle_authority_supplies_all_fixture_templates():
    from bioxp.oem_machine_bundle import load_oem_machine_snapshot
    bundle = Path(MANIFEST['templates/cover.jpg']['source']).parent.parent
    snapshot = load_oem_machine_snapshot(bundle / 'OEM_EVIDENCE_LOCK.json')
    assert snapshot.machine_serial == 206 and snapshot.camera_calibrated
    for name in TEMPLATES:
        source = snapshot.records['appdata/' + Path(name).name]
        assert source.raw_bytes == fixture_bytes(name)
        assert source.sha256 == MANIFEST[name]['sha256']
    for name in ('InspectionSettings.xml', 'config.xml', 'calreference.xml'):
        assert snapshot.records['appdata/' + name].raw_bytes == fixture_bytes('configuration/' + name)
    record('immutable-bundle', {'lock_sha256': snapshot.lock_sha256,
           'serial': snapshot.machine_serial, 'profile': snapshot.inspection_profile_name,
           'template_count': len(TEMPLATES), 'camera_calibrated': snapshot.camera_calibrated})


def test_check_label_two_exposures_real_frame_transformation_controls():
    raw = fixture_bytes('snapshots/output_plate_missing_2018-10-09 11-52-42.jpg')
    dark = cv2.cvtColor(cv2.imdecode(np.frombuffer(raw, np.uint8), 1), 7)
    assert vision.label_dark_frame_valid(dark)
    assert not vision.check_label(dark, dark)  # stale identical frame is rejected
    # Deterministic spatially permuted authentic image pixels, then brighten:
    # algorithm control, not a claimed recorded LED-on exposure.
    bright = np.random.default_rng(206).permutation(dark.ravel()).reshape(dark.shape)
    bright = np.clip(bright.astype(np.int16) + 60, 0, 254).astype(np.uint8)
    assert vision.check_label(dark, bright)
    assert not vision.check_label(dark, np.minimum(bright, 49))
    uniform_histogram = np.tile(np.arange(256, dtype=np.uint8), (480, 3))[:, :640]
    assert not vision.label_dark_frame_valid(uniform_histogram)
    assert not vision.check_label(uniform_histogram, bright)
    record('check-label-transform', {'same_frame': False, 'permuted_brightened': True,
           'too_dark_on': False, 'flat_histogram_off': False,
           'qualification': 'saved OEM pixels with labeled transformations; no paired LED exposure capture available'})


def test_method_zero_preserves_oem_maximum_not_minimum():
    gray = cv2.imdecode(np.frombuffer(fixture_bytes('templates/EmptyTrough.jpg'), np.uint8), 0)
    image = np.pad(gray, ((0, 0), (0, gray.shape[1])))
    actual = vision.match_pattern(image, gray, 0)
    result = cv2.matchTemplate(image, gray, 0)
    minimum, maximum, minloc, maxloc = cv2.minMaxLoc(result)
    assert minimum < maximum and minloc != maxloc
    assert actual == (maximum, *maxloc)


def test_color_conversion_uses_source_numeric_code_seven():
    bgr = np.zeros((480, 640, 3), np.uint8)
    bgr[:, :, 0] = 255
    assert cv2.cvtColor(bgr, 7)[0, 0] == 76
    assert cv2.cvtColor(bgr, 6)[0, 0] == 29
    assert vision.check_purification_station(bgr)
    assert not vision.check_purification_station(cv2.cvtColor(bgr, 6))


@pytest.mark.parametrize('which,boundary', [('purification', 20000), ('output', 1000), ('trough', 2000)])
def test_pixel_threshold_boundaries_and_roi_polarity(which, boundary):
    gray = np.full((480, 640), 0, np.uint8)
    if which == 'purification':
        roi = gray[:240, :]
        threshold = 40
        check = vision.check_purification_station
        expected_at = False
    elif which == 'output':
        roi = gray[240:, :320]
        threshold = configured('OutputPlateInspection', 'threshold')
        check = lambda x: vision.check_output_plate(x, threshold, boundary)
        expected_at = True
    else:
        roi = gray[240:430, 320:]
        threshold = configured('TroughInspection', 'threshold')
        check = lambda x: vision.check_trough(x, threshold)
        expected_at = True
    coords = np.unravel_index(np.arange(boundary + 1), roi.shape)
    roi[coords] = threshold
    assert not check(gray)
    roi[coords[0][:-1], coords[1][:-1]] = threshold + 1
    assert check(gray) == expected_at
    roi[coords[0][-1], coords[1][-1]] = threshold + 1
    assert check(gray)
    assert not check(np.flip(gray, axis=0))


def test_notch_status_all_branches_and_ieee_zero_division():
    image = np.zeros((480, 640), np.uint8)
    assert vision.check_notch(image) == 1  # 0/0 NaN, not an exception
    image[220:280, 200:260] = 100
    image[220:280, 290:350] = 100
    image[130:190, 290:350] = 10
    assert vision.check_notch(image) == 2
    image[220:280, 290:350] = 10
    assert vision.check_notch(image) == 0
    image[130:190, 290:350] = 100
    assert vision.check_notch(image) == 1


def test_histogram_white_exclusion_and_cover_polarity():
    image = np.full((480, 640), 255, np.uint8)
    assert vision._hist(image).sum() == 0
    assert vision.locate_cover(image)  # literal OEM empty-histogram behavior
    image[:] = 10
    assert not vision.locate_cover(image)
    assert not vision.check_biosecurity_cover(image)
    image[:300] = 180
    assert vision.locate_cover(image)
    assert vision.check_biosecurity_cover(image)


def test_caller_threshold_and_tie_precedence():
    assert vision.output_location_from_scores(.8, .8, .8, .8) == 1
    assert vision.output_location_from_scores(.1, .2, .1, .1) == 2  # no gate on plain output
    assert vision.output_location_from_scores(.1, .1, .7, .1) == 0
    assert vision.output_location_from_scores(.1, .1, .70001, .1) == 2
    assert vision.reagent_cover_from_scores(-.1, -.1, -.2)
    assert not vision.reagent_cover_from_scores(.7, .8, .1)


def test_blobs_exact_eight_connectivity_area_and_border_rules():
    image = np.zeros((120, 160), np.uint8)
    image[0:20, 10:35] = 255  # top border is allowed; exactly 500 pixels
    image[20:40, 35:60] = 255  # diagonal-only contact merges in 8-connectivity
    image[50:75, :20] = 255   # x=0 component excluded
    image[80:100, 80:104] = 255  # 480 pixels excluded
    actual = tips.get_blobs(image)
    assert len(actual) == 1
    assert actual[0] == tips.Blob(10, 59, 0, 39, 34, 19, 1000)


def transformed_tip_frame(radius, removed=()):
    raw = fixture_bytes('snapshots/tip-tray-1_2021-07-26-10-39-33.jpg')
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), 0)
    assert image.shape == (480, 640)
    # Explicit computational control, NOT an OEM-authored inventory annotation:
    # retain original 20px side margins; replace inspection ROI then stamp holes.
    image[:, 20:-20] = 180
    for i, (x, y) in enumerate(tips.TIP_CENTERS):
        if i not in removed:
            cv2.circle(image, (x + 20, y), radius, 0, -1)
    return image


@pytest.mark.parametrize('tiptype,radius', [(50, 40), (200, 45)])
def test_actual_frame_transformed_hole_masks_and_negative_control(tiptype, radius):
    image = transformed_tip_frame(radius)
    threshold = configured('TipInspection', 'threshold')
    assert tips.missing_tips(image, threshold, tiptype) == 0xFFFF00
    missing_one = transformed_tip_frame(radius, removed=(5,))
    assert tips.missing_tips(missing_one, threshold, tiptype) == (0xFFFF ^ (1 << 5)) << 8
    cv2.imwrite(str(Path(os.environ['OEM_CV_RESULTS']) / f'tip-{tiptype}-transformed-control.png'), image)
    if tiptype == 50:
        assert tips.missing_tips(transformed_tip_frame(radius, removed=(5, 6)), threshold, 50) == -1
        assert tips.missing_tips(transformed_tip_frame(20), threshold, 50) == 0
    else:
        assert tips.missing_tips(transformed_tip_frame(40), threshold, 200) == 0  # bbox below 86


@pytest.mark.parametrize('region', range(6))
def test_every_region_bit_maps_to_real_source_row_major_well(region):
    expected = [(0, 0), (0, 4), (0, 8), (4, 8), (4, 4), (4, 0)][region]
    for bit in range(16):
        col_offset, reversed_row = divmod(bit, 4)
        row = expected[0] + 3 - reversed_row
        column = expected[1] + col_offset
        assert tips.missing_tip_wells(1 << (bit + 8), region) == (row * 12 + column,)
    assert len(tips.missing_tip_wells(0xFFFF00, region)) == 16
    assert tips.missing_tip_wells(0, region) == ()


def test_all_region_membership_exactly_96_unique_wells():
    wells = [well for r in range(6) for well in tips.missing_tip_wells(0xFFFF00, r)]
    assert len(wells) == len(set(wells)) == 96
    assert sorted(wells) == list(range(96))


@pytest.mark.parametrize('bad', [b'', b'not an image', np.empty((0, 0), np.uint8), np.ones((10, 10), np.float32)])
def test_invalid_images_never_become_success_or_empty_inventory(bad):
    with pytest.raises(ValueError):
        vision.check_purification_station(bad)
    with pytest.raises(ValueError):
        tips.missing_tips(bad, 30, 50)


def test_invalid_roi_mask_and_method_rejected():
    with pytest.raises(ValueError):
        vision.check_trough(np.zeros((100, 100), np.uint8), 80)
    with pytest.raises(ValueError):
        vision.check_notch(np.zeros((100, 100), np.uint8))
    with pytest.raises(ValueError):
        tips.missing_tip_wells(-1, 0)
    with pytest.raises(ValueError):
        vision.match_pattern(np.zeros((10, 10), np.uint8), np.zeros((20, 20), np.uint8))
    with pytest.raises(ValueError):
        vision.match_pattern(np.zeros((10, 10), np.uint8), np.zeros((5, 5), np.uint8), 6)
