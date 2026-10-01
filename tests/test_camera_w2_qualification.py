"""W2 frozen-image qualification: real source-copy reader, no devices/network."""
import asyncio
import base64
from datetime import datetime, timezone
import hashlib
import inspect
import io
import json
import os
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from PIL import Image, JpegImagePlugin
import pytest
from starlette.requests import Request
from src.bioxp.camera_provider import CameraProvider, CameraUnavailable, CameraFrameUnavailable
from tests.test_camera_capture_cadence import rig
from tests.test_camera_post_request_capture import provider
from tests.test_camera_oem_inspection import rig as inspection_rig

_REAL_SPAWN = asyncio.create_subprocess_exec

def jpeg(mode='RGB', size=(640,480), **opts):
    pixels=np.random.default_rng(113).integers(0,256,(size[1],size[0],3),dtype=np.uint8)
    out=io.BytesIO();Image.fromarray(pixels).convert(mode).save(out,'JPEG',**opts);return out.getvalue()

def trace_loads(monkeypatch):
    records=[]; original=JpegImagePlugin.JpegImageFile.load
    def load(image,*args,**kwargs):
        before=(image.mode,image.size)
        result=original(image,*args,**kwargs)
        records.append({'before':before,'after':(image.mode,image.size),'raster':(image.im.mode,image.im.size)})
        return result
    monkeypatch.setattr(JpegImagePlugin.JpegImageFile,'load',load)
    return records

def optimized():
    return 'reduced_stream' in inspect.signature(CameraProvider._validate_jpeg).parameters

@pytest.mark.parametrize('sampling',[0,1,2])
@pytest.mark.parametrize('layout',['normal','restart','metadata'])
def test_w2_actual_materialization_and_original_bytes(monkeypatch,sampling,layout):
    opts={'subsampling':sampling}
    if layout=='restart':opts['restart_marker_blocks']=4
    if layout=='metadata':opts['exif']=b'Exif\x00\x00' + b'metadata'*100
    content=jpeg(**opts); records=trace_loads(monkeypatch); camera=provider()
    try:
        first=camera.publish_stream_frame('owner',content,source_captured_at=datetime.now(timezone.utc))
        assert first.content is content and first.content_sha256==hashlib.sha256(content).hexdigest()
        assert records[-1]['raster']==(('L',(80,60)) if optimized() else ('RGB',(640,480)))
        count=len(records);camera.publish_stream_frame('owner',bytes(bytearray(content)))
        assert len(records)==count # Inherited duplicate skipping, not W2 savings.
        camera._validate_jpeg(content)
        assert records[-1]['raster']==('RGB',(640,480)) # Default nonstream/inspection.
        camera.publish_stream_frame('owner',bytearray(content))  # type: ignore[arg-type]  # Deliberately untrusted input.
        assert records[-1]['raster']==('RGB',(640,480)) # Mutable/untrusted full decode.
        camera.end_stream('owner');camera.begin_stream('replacement')
        with pytest.raises(CameraFrameUnavailable):camera.publish_stream_frame('owner',content)
        camera.publish_stream_frame('replacement',content)
        assert records[-1]['raster']==(('L',(80,60)) if optimized() else ('RGB',(640,480)))
    finally:camera.end_stream('replacement');camera.end_stream('owner')
    print({'sampling':sampling,'layout':layout,'loads':records})

@pytest.mark.parametrize('mode,opts',[('L',{}),('CMYK',{}),('RGB',{'progressive':True}),('RGB',{'keep_rgb':True,'subsampling':0})])
def test_w2_unqualified_encoding_full_fallback(monkeypatch,mode,opts):
    content=jpeg(mode,**opts);records=trace_loads(monkeypatch);camera=provider()
    try:
        frame=camera.publish_stream_frame('owner',content)
        assert frame.content==content and records[-1]['raster']==(mode,(640,480))
    finally:camera.end_stream('owner')

@pytest.mark.parametrize('size',[(80,60),(320,240),(1280,960),(640,481)])
def test_w2_original_dimensions_checked_before_draft(monkeypatch,size):
    content=jpeg(size=size); records=trace_loads(monkeypatch);camera=provider()
    try:
        with pytest.raises(CameraUnavailable,match='not a 640x480'):camera.publish_stream_frame('owner',content)
        assert not records and not camera.status().available
    finally:camera.end_stream('owner')

def test_w2_real_nonstream_and_inspection_full_materialization(inspection_rig,monkeypatch):
    from src.bioxp.camera_provider import OemInspectionCameraSettings
    camera,state=inspection_rig;content=jpeg();state.frames=content
    records=trace_loads(monkeypatch)
    assert camera.capture().content==content
    state.frames=content*2
    assert camera.capture_inspection(OemInspectionCameraSettings()).frame.content==content
    assert len(records)>=3 and all(r['raster']==('RGB',(640,480)) for r in records)
    print({'full_capture_inspection_loads':records})


def test_w2_connected_owner_replacement_with_queued_metadata(rig,monkeypatch):
    api,camera,processes,_=rig;content=jpeg();records=trace_loads(monkeypatch)
    async def scenario():
        await api._start_owned_camera_session({});old=api._camera_session;old_owner=old['session_id'];generation=camera.generation
        # A blocked reader and full sideband queue must not survive teardown.
        proc=processes[0];proc.stderr.feed_data(b'#tb 0: 1/1000000\n'+f'0, 0, 0, 1, {len(content)}, 0x0\n'.encode()*20)
        await asyncio.sleep(.02)
        await asyncio.wait_for(api._stop_owned_camera_session(reason='W2 queued metadata owner replacement'),1)
        assert proc.returncode is not None
        await api._start_owned_camera_session({});new=api._camera_session;fresh=processes[1]
        assert new is not old and camera.generation!=generation
        with pytest.raises(CameraFrameUnavailable):camera.publish_stream_frame(old_owner,content)
        pts=int(datetime.now(timezone.utc).timestamp()*1000000)
        fresh.stderr.feed_data(b'#tb 0: 1/1000000\n'+f'0, {pts}, {pts}, 1, {len(content)}, 0x0\n'.encode());fresh.stdout.feed_data(content)
        try:
            for _ in range(1000):
                if camera.status().available:break
                await asyncio.sleep(.001)
            assert camera.latest().content==content and api._camera_session is new
            assert camera.latest().source_captured_at.timestamp()==pts/1000000
            assert records[-1]['raster']==(('L',(80,60)) if optimized() else ('RGB',(640,480)))
        finally:await api._stop_owned_camera_session(reason='W2 replacement cleanup')
        assert all(p.returncode is not None for p in processes) and camera._stream_owner is None
    asyncio.run(scenario())


def damaged(kind):
    good=jpeg(quality=95)
    if kind=='quant':
        b=bytearray(good); b[b.index(b'\xff\xc0')+12]=3; return bytes(b)
    if kind=='wrong-dim':return jpeg(size=(320,240))
    if kind=='truncated':return good[:-2]
    if kind=='entropy-cut':
        sos=good.index(b'\xff\xda');start=sos+2+int.from_bytes(good[sos+2:sos+4],'big')
        return good[:start+(len(good)-start)//2]+b'\xff\xd9'
    if kind=='entropy-reject':
        prior=json.loads(Path('/prior/probe-results.json').read_text())
        row=next(r for r in prior['rows'] if ':entropy-bitflip:' in r['name'] and not r['results']['baseline']['accepted'])
        b=bytearray((Path('/corpus/fixtures')/row['parent']).read_bytes());b[row['mutation']['offset']]=row['mutation']['new']
        assert hashlib.sha256(b).hexdigest()==row['sha256'];return bytes(b)
    raise AssertionError(kind)

@pytest.mark.parametrize('kind',['quant','entropy-reject','entropy-cut','wrong-dim','truncated'])
def test_w2_real_ffmpeg_invalid_recovery_snapshot_analysis(rig,monkeypatch,kind):
    api,camera,processes,_=rig
    first=jpeg(quality=95); bad=damaged(kind)
    out=io.BytesIO();Image.new('RGB',(640,480),'blue').save(out,'JPEG');recovery=out.getvalue()
    assert len(first)>65536
    try:CameraProvider._validate_jpeg(bad); accepted=True
    except CameraUnavailable:accepted=False
    loads=trace_loads(monkeypatch)
    records=[];tasks=[];commands=[]
    original=camera.publish_stream_frame
    def observed(owner,content,**kwargs):
        try:
            frame=original(owner,content,**kwargs)
            records.append((content,True,frame.sequence,frame.source_captured_at));return frame
        except CameraUnavailable:
            assert not camera.status().available
            records.append((content,False,camera._sequence,kwargs.get('source_captured_at')));raise
    monkeypatch.setattr(camera,'publish_stream_frame',observed)
    release=asyncio.Event()
    async def spawn(*argv,**kwargs):
        output=list(argv[argv.index('-i')+2:])
        assert output[output.index('-c:v')+1]=='copy'
        assert output[-1].startswith('[f=framecrc:flush_packets=1]pipe:2|')
        cmd=[os.environ['CAMERA_TEST_FFMPEG'],'-hide_banner','-loglevel','error','-copyts','-probesize','32','-analyzeduration','0','-use_wallclock_as_timestamps','1','-f','mjpeg','-framerate','30','-i','pipe:0',*output]
        commands.append(cmd);proc=await _REAL_SPAWN(*cmd,stdin=asyncio.subprocess.PIPE,**kwargs);processes.append(proc)
        stdin=proc.stdin;assert stdin is not None
        async def write():
            try:
                for content,count in [(first,8),(bad,8)]:
                    for _ in range(count):stdin.write(content);await stdin.drain();await asyncio.sleep(.05)
                await release.wait()
                for _ in range(100):stdin.write(recovery);await stdin.drain();await asyncio.sleep(.05)
            except (BrokenPipeError,ConnectionResetError):pass
            finally:stdin.close()
        tasks.append(asyncio.create_task(write()));return proc
    monkeypatch.setattr(api.asyncio,'create_subprocess_exec',spawn)
    # Generic route service/capture is real; only hardware capability discovery is doubled.
    monkeypatch.setattr(api,'_get_vision_capabilities',lambda:SimpleNamespace(is_enabled=lambda _:True))
    async def blocking(_label,fn,**_):return await asyncio.to_thread(fn)
    monkeypatch.setattr(api,'_run_blocking',blocking)
    async def wait_for(predicate):
        for _ in range(5000):
            if predicate():return
            await asyncio.sleep(.001)
        pytest.fail('reader did not reach expected controlled stage')
    async def scenario():
        await api._start_owned_camera_session({});owner=api._camera_session;generation=camera.generation
        response=await api.camera_mjpeg(Request({'type':'http','query_string':b''}));iterator=response.body_iterator
        parts=[]
        async def consume():
            async for part in iterator:parts.append(part)
        consumer=asyncio.create_task(consume())
        try:
            await wait_for(lambda:any(r[0]==bad for r in records))
            pending=asyncio.create_task(api.camera_snapshot())
            await wait_for(lambda:sum(r[0]==bad for r in records)>=6)
            await asyncio.sleep(.02)
            if not accepted:
                assert not pending.done() and not camera.status().available
                assert camera.status().dropped_frames>=6
                # A missing-EOI packet is a prefix of its valid original;
                # compare exact multipart payloads, not substring containment.
                assert all(p.partition(b'\r\n\r\n')[2][:-2] != bad for p in parts)
            release.set();snapshot=await asyncio.wait_for(pending,5)
            assert snapshot.body in ([bad,recovery] if accepted else [recovery])
            await wait_for(lambda:any(r[0]==recovery for r in records))
            generic=await asyncio.wait_for(api.vision_inspect(api.InspectionRequest()),5)
            encoded=base64.b64decode(generic['snapshot']['image_b64']);assert encoded==recovery
            # Downstream pixels remain full color, equal to independent original decode.
            with Image.open(io.BytesIO(encoded)) as image:
                image.load();assert image.mode=='RGB' and image.size==(640,480);pixels=np.array(image)
            with Image.open(io.BytesIO(recovery)) as image:np.testing.assert_array_equal(pixels,np.array(image))
            from src.bioxp.vision.oem_inspection import _pixels
            decoded=_pixels(encoded); assert decoded.shape==(480,640,3)
            np.testing.assert_array_equal(decoded,_pixels(recovery))
            await wait_for(lambda:any(recovery in p for p in parts))
            assert camera.generation==generation and api._camera_session is owner
            assert all(r[0] in [first,bad,recovery] for r in records)
            assert all(r[1]==accepted for r in records if r[0]==bad)
            assert all(r[3] is not None for r in records)
            raster_types={r['raster'] for r in loads}
            assert ('RGB',(640,480)) in raster_types # Real full downstream analysis.
            if optimized():assert ('L',(80,60)) in raster_types
            else:assert ('L',(80,60)) not in raster_types
            print({'kind':kind,'baseline_accepted':accepted,'observed_packets':len(records),'bad_packets':sum(r[0]==bad for r in records),'multipart_parts':len(parts),'materialized':loads,'analysis_pixel_sha256':hashlib.sha256(pixels.tobytes()).hexdigest(),'argv':commands[0]})
        finally:
            consumer.cancel();await asyncio.gather(consumer,return_exceptions=True);await iterator.aclose()
            await api._stop_owned_camera_session(reason='W2 controlled offline cleanup')
            for task in tasks:task.cancel()
            await asyncio.gather(*tasks,return_exceptions=True)
        assert len(processes)==1 and processes[0].returncode is not None and camera._stream_owner is None
    asyncio.run(scenario())

@pytest.mark.parametrize('packet_kind',['wrong-format','oversized-packet','oversized-metadata'])
def test_w2_connected_bounds_match_existing_reader(rig,packet_kind):
    api,camera,processes,_=rig
    async def scenario():
        await api._start_owned_camera_session({});proc=processes[0]
        b=io.BytesIO();Image.new('RGB',(640,480)).save(b,'PNG');content=b'\xff\xd8'+b.getvalue()+b'\xff\xd9'
        if packet_kind=='oversized-packet':content=b'\xff\xd8'+b'x'*(2*1024*1024)+b'\xff\xd9'
        proc.stderr.feed_data(b'#tb 0: 1/1000000\n')
        if packet_kind=='oversized-metadata':proc.stderr.feed_data(b'X'*100000+b'\n')
        else:
            pts=int(datetime.now(timezone.utc).timestamp()*1000000)
            proc.stderr.feed_data(f'0, {pts}, {pts}, 33333, {len(content)}, 0x0\n'.encode());proc.stdout.feed_data(content)
        try:
            for _ in range(1000):
                if (packet_kind=='wrong-format' and camera.status().dropped_frames) or (packet_kind!='wrong-format' and proc.returncode is not None):break
                await asyncio.sleep(.001)
            assert not camera.status().available
            if packet_kind=='wrong-format':
                assert camera.status().dropped_frames==1
                good=jpeg();seq=camera._sequence;pts=int(datetime.now(timezone.utc).timestamp()*1000000)
                proc.stderr.feed_data(f'0, {pts}, {pts}, 33333, {len(good)}, 0x0\n'.encode());proc.stdout.feed_data(good)
                for _ in range(1000):
                    if camera._sequence>seq:break
                    await asyncio.sleep(.001)
                assert camera.latest().content==good
            else:assert proc.returncode is not None and camera._stream_owner is None
        finally:await api._stop_owned_camera_session(reason='W2 existing bounded reader cleanup')
    asyncio.run(scenario())
