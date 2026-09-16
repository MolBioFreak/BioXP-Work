"""Strip-only computational qualification, real archived pixels + labeled controls.

No live camera/robot test. External immutable fixtures required (never skip).
"""
import hashlib
import json
import math
import os
from pathlib import Path

import cv2
import numpy as np
import pytest

from bioxp.vision import oem_strips as s

E = Path(os.environ['OEM_CV_EVIDENCE'])
MANIFEST = json.loads((E / 'fixture-manifest.json').read_text())
FRAMES = sorted(n for n in MANIFEST if n.startswith('snapshots/'))
HANDLES = ['templates/LowerHandle.jpg', 'templates/UpperHandle.jpg']


def raw(name):
    value = (E / 'fixtures' / name).read_bytes()
    assert hashlib.sha256(value).hexdigest() == MANIFEST[name]['sha256']
    assert hashlib.sha256(Path(MANIFEST[name]['source']).read_bytes()).hexdigest() == MANIFEST[name]['sha256']
    return value


def record(name, value):
    destination = Path(os.environ['OEM_CV_RESULTS'])
    destination.mkdir(exist_ok=True)
    (destination / (name + '.json')).write_text(json.dumps(value, indent=2) + '\n')


def morphology_reference(gray, y):
    # Independent literal 3x3 min/max windows; preserve C++ parent ROI alias.
    parent = gray.copy()
    scratch = None
    for index, dilate in enumerate((True, True, False, False, False, False)):
        source = parent if index % 2 == 0 else scratch
        padded = np.pad(source, 1, constant_values=0 if dilate else 255)
        neighborhoods = np.lib.stride_tricks.sliding_window_view(padded, (3, 3))
        value = (neighborhoods.max if dilate else neighborhoods.min)(axis=(-1, -2))
        if index % 2 == 0:
            scratch = value[y:y + 100].copy()
        else:
            parent[y:y + 100] = value
    return parent[y:y + 100]


def bits_reference(lines):
    bits = 0
    for line in (() if lines is None else lines.reshape(-1, 4)):
        a, b, c, d = map(int, line)
        angle = float(np.float32(math.atan2(float(np.float32(d-b)), float(np.float32(c-a)))))
        if a == 549 or c == 549:
            continue
        if min(abs(angle-1.57), abs(angle-4.712), abs(angle+1.57), abs(angle+4.712)) < .174:
            for x, mask in zip((56,186,316,446,576), (16,8,4,2,1)):
                if abs(a-x) < 30:
                    bits |= mask
                    break
    return bits


def strip_reference(bgr, low, high, equalize):
    gray = cv2.cvtColor(bgr, 6)
    if equalize:
        gray = cv2.equalizeHist(gray)
    result = 0
    for y in (1,127,253,379):
        band = morphology_reference(gray, y)
        gray[y:y+100] = band
        edges = cv2.Canny(band, int(low), int(high), apertureSize=3, L2gradient=False)
        lines = cv2.HoughLinesP(edges, 1, math.pi/180, 30, minLineLength=20, maxLineGap=10)
        result = (result << 8) | bits_reference(lines)
    return result


def handle_reference(bgr, adjustment, equalize, file_mode):
    gray = cv2.cvtColor(bgr, 6 if file_mode else 7)
    if equalize:
        gray = cv2.equalizeHist(gray)
    crop = cv2.medianBlur(gray[110:370,210+adjustment:430+adjustment],5)
    hist = cv2.calcHist([crop],[0],None,[256],[0,255]).ravel()
    peak = 0
    valley = float('inf')
    selected = 0
    search = False
    for i in reversed(range(252)):
        count = float(hist[i])
        if not search:
            if count > peak:
                peak = count
            elif count < peak/2 and peak > 250:
                search = True
        if search:
            if count < valley:
                valley, selected = count, i
            elif count > valley*25:
                break
    selected = 180 if selected < 100 else selected
    # Independent OpenCV connected-component oracle (not production OEM BFS).
    n, labels, stats, centers = cv2.connectedComponentsWithStats(cv2.inRange(crop,0,selected),8)
    best = (0,0,0)
    eligible = []
    for label in range(1,n):
        x,y,w,h,area = map(int,stats[label])
        if area >= 500 and x != 0 and x+w != 220:
            yy,xx = np.nonzero(labels == label)
            interior = (xx > 0) & (xx < 219) & (yy > 0) & (yy < 259)
            if interior.any():
                seed = int(np.min(yy[interior]*220+xx[interior]))
                eligible.append((seed,area,int(xx.sum())//area,int(yy.sum())//area))
    for _,area,x,y in sorted(eligible):
        if area > best[0]:
            best = area,x,y
    return best


@pytest.mark.parametrize('name',FRAMES)
@pytest.mark.parametrize('equalize',[False,True])
def test_archived_pixels_both_computations_independent_oracles(name,equalize):
    encoded = raw(name)
    bgr = cv2.imdecode(np.frombuffer(encoded,np.uint8),1)
    # These assets are full-size original OEM frames, never resized.
    assert bgr.shape[0] >= 479 and bgr.shape[1] >= 520
    expected_strip = strip_reference(bgr,50,150,equalize)
    actual_strip = s.inspect_strip(encoded,50,150,equalize)
    assert actual_strip == expected_strip
    assert s.inspect_strip(encoded,50,150,equalize,file_mode=True) == expected_strip
    handles = {}
    for adjustment in (0,30,60,90,-30,-60,-90):
        expected = handle_reference(bgr,adjustment,equalize,False)
        actual = s.find_strip_handle(encoded,adjustment,equalize)
        assert actual == expected
        handles[str(adjustment)] = actual
    # JPEG decoder0 differs from cvtColor6; use its actual decoded array oracle.
    file_gray = cv2.imdecode(np.frombuffer(encoded,np.uint8),0)
    gray_bgr = cv2.cvtColor(file_gray,cv2.COLOR_GRAY2BGR)
    assert s.find_strip_handle(encoded,0,equalize,file_mode=True) == handle_reference(gray_bgr,0,equalize,True)
    record(Path(name).stem + '-' + str(equalize), {'fixture':name,'shape':bgr.shape,
           'equalize':equalize,'handle_results':handles,'strip_mask':actual_strip,
           'scope':'archived-pixel computation, no physical strip ground truth'})


@pytest.mark.parametrize('name',HANDLES)
@pytest.mark.parametrize('equalize',[False,True])
def test_archived_handle_template_explicit_padding_control(name,equalize):
    encoded = raw(name)
    template = cv2.imdecode(np.frombuffer(encoded,np.uint8),1)
    assert template.shape == (144,144,3)
    # Original templates are NOT full camera frames. Reject them unchanged.
    with pytest.raises(ValueError):
        s.find_strip_handle(encoded,0,equalize)
    with pytest.raises(ValueError):
        s.inspect_strip(encoded,50,150,equalize)
    # Exact template pixels padded at x248/y150; explicit transformed control.
    padded = np.pad(template,((150,186),(248,248),(0,0)),constant_values=255)
    handle = s.find_strip_handle(padded,0,equalize)
    strip = s.inspect_strip(padded,50,150,equalize)
    assert handle == handle_reference(padded,0,equalize,False)
    assert strip == strip_reference(padded,50,150,equalize)
    record(Path(name).stem+'-padded-'+str(equalize),{'fixture':name,
           'transformation':'unchanged template padded white at x248/y150 to 640x480',
           'handle':handle,'strip':strip,'equalize':equalize})


@pytest.mark.parametrize('adjustment',[-90,-60,-30,0,30,60,90])
def test_transformed_handle_geometry_roi_coordinates_and_input_unchanged(adjustment):
    # Explicit controlled replacement of archived frame pixels, NOT live image.
    frame = cv2.imdecode(np.frombuffer(raw(FRAMES[0]),np.uint8),1)
    frame[:] = 255
    frame[150:190,260+adjustment:300+adjustment] = 0
    before = frame.copy()
    value = s.find_strip_handle(frame,adjustment,False)
    assert value == (1588,69,59)  # 40x40 minus twelve median-filter corners.
    np.testing.assert_array_equal(frame,before)
    assert s.find_strip_handle(255-frame,adjustment,False) == (0,0,0)


def test_transformed_strip_distinct_band_packing_and_threshold_conversion():
    frame = cv2.imdecode(np.frombuffer(raw(FRAMES[0]),np.uint8),1)
    frame[:] = 0
    for y,x in zip((1,127,253,379),(56,186,316,576)):
        frame[y:y+100,x:x+16] = 255
    before = frame.copy()
    assert s.inspect_strip(frame,50.9,150.9,False) == 0x10080401
    assert s.inspect_strip(frame,50,150,True) == 0x10080401
    assert s.inspect_strip(frame,5000,6000,False) == 0
    np.testing.assert_array_equal(before,frame)


@pytest.mark.parametrize('center,bit',[(56,16),(186,8),(316,4),(446,2),(576,1)])
def test_line_position_strict_boundary_and_direction(center,bit):
    for x in (center-29,center,center+29):
        assert s._strip_line_bits(np.array([[[x,0,x,99]]])) == bit
        assert s._strip_line_bits(np.array([[[x,99,x,0]]])) == bit
    for x in (center-30,center+30):
        assert s._strip_line_bits(np.array([[[x,0,x,99]]])) == 0
    assert s._strip_line_bits(np.array([[[center,0,center+50,0]]])) == 0


def test_source_549_exclusion_checks_both_endpoints():
    assert s._strip_line_bits(np.array([[[549,0,549,99]]])) == 0
    assert s._strip_line_bits(np.array([[[550,0,549,99]]])) == 0
    assert s._strip_line_bits(np.array([[[550,0,550,99]]])) == 1


def test_threshold_valley_strict_comparisons_and_fallback():
    h = np.zeros(256,np.float32)
    assert s._handle_threshold(h) == 180
    h[230] = 250
    assert s._handle_threshold(h) == 180
    h[230] = 251
    assert s._handle_threshold(h) == 229
    h[:] = 0
    h[252:] = 10000
    assert s._handle_threshold(h) == 180
    h[99] = 1000
    assert s._handle_threshold(h) == 180


def test_morphology_parent_border_not_isolated_or_collapsed():
    frame = cv2.imdecode(np.frombuffer(raw(FRAMES[0]),np.uint8),0)
    for y in (1,127,253,379):
        expected = morphology_reference(frame,y)
        parent = frame.copy()
        actual = s._strip_morphology(parent,y)
        np.testing.assert_array_equal(actual,expected)
        isolated = cv2.erode(cv2.dilate(frame[y:y+100],None,iterations=2),None,iterations=4)
        assert np.any(actual != isolated)


@pytest.mark.parametrize('function,args',[(s.find_strip_handle,(0,False)),(s.inspect_strip,(50,150,False))])
def test_invalid_inputs_raise_not_absence(function,args):
    for invalid in (b'',b'not an image',np.zeros((20,20),np.uint8)):
        with pytest.raises((ValueError,cv2.error)):
            function(invalid,*args)


def test_invalid_numeric_parameters():
    image = cv2.imdecode(np.frombuffer(raw(FRAMES[0]),np.uint8),1)
    for invalid in (True,float('nan'),float('inf'),2**31):
        with pytest.raises(ValueError):
            s.inspect_strip(image,invalid,100,False)
    with pytest.raises(ValueError):
        s.find_strip_handle(image,1.5,False)
    with pytest.raises(ValueError):
        s.find_strip_handle(image,-211,False)


def test_captured_settings_native_keyword_interface():
    import xml.etree.ElementTree as ET
    root = ET.fromstring(raw('configuration/InspectionSettings.xml'))
    section = root.find('{*}Settings3200')
    entry = next(row for row in section if row.find('{*}Key').text == 'StripInspection')
    parameters = {p.find('{*}Key').text:p.find('{*}Value').text
                  for p in entry.find('{*}Value').find('{*}Parameters')}
    args = {'threLow':float(parameters['threLow']), 'threHigh':float(parameters['threHigh']),
            'equalize':parameters['equalize'] == 'true'}
    image = raw(FRAMES[0])
    bgr = cv2.imdecode(np.frombuffer(image,np.uint8),1)
    actual = s.inspect_strip(image,**args)
    assert actual == strip_reference(bgr,args['threLow'],args['threHigh'],args['equalize'])
    assert s.find_strip_handle(image,**{'adjustment':0,'equalize':False}) == handle_reference(bgr,0,False,False)
    record('native-keyword-interface',{'settings':args,'strip_mask':actual})


def test_runtime_is_qualified_pin_and_isolated():
    assert cv2.__version__ == '3.4.18'
    assert np.__version__ == '1.26.4'
    assert len(Path('/proc/net/route').read_text().splitlines()) == 1
    assert not any(p.name.startswith(('video','ttyUSB','ttyACM','can')) for p in Path('/dev').iterdir())
