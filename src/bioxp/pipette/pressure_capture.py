"""Opt-in bounded scientific pressure acquisition, not operational bulk logging.

Receiver only offers immutable frames to bounded memory with a nonblocking lock.
Decoding, statistics and SQLite writes run on the finite caller, never receiver.
No stream command, baseline correction, threshold or anomaly model is implied.
"""
import statistics
import threading
import time
from typing import Annotated
from pydantic import BaseModel, ConfigDict, Field, field_validator


class PressureCaptureRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    channels: list[Annotated[int, Field(ge=0, le=3)]] = Field(min_length=1, max_length=4)
    max_frames: int = Field(ge=1, le=4096)
    duration_ms: int = Field(ge=1, le=120000)
    sample_period_ms: float | None = Field(default=None, gt=0)

    @field_validator('channels')
    @classmethod
    def unique_channels(cls, value):
        if len(set(value)) != len(value):
            raise ValueError('capture channels must be unique')
        return value


class ScientificPressureCapture:
    def __init__(self, request, *, reader_generation, clock=time.monotonic):
        self.request = PressureCaptureRequest.model_validate(request)
        if len(set(self.request.channels)) != len(self.request.channels) or any(c not in range(4) for c in self.request.channels):
            raise ValueError('capture channels must be unique integers within 0..3')
        self.reader_generation = reader_generation
        self.clock = clock
        self.started_at = clock()
        self.deadline = self.started_at + self.request.duration_ms / 1000
        self.frames = []
        self.lock = threading.Lock()
        self.closed = False
        self.lost_frames = 0

    def offer(self, frame):
        if self.closed or frame.received_at < self.started_at or frame.received_at > self.deadline:
            return
        if ((frame.arbitration_id & 0x78) >> 3) not in self.request.channels:
            return
        if not self.lock.acquire(blocking=False):
            self.lost_frames += 1
            return
        try:
            if self.closed:
                return
            if len(self.frames) >= self.request.max_frames:
                self.lost_frames += 1
            else:
                self.frames.append(frame)
        finally:
            self.lock.release()

    def finish(self):
        with self.lock:
            self.closed = True
            frames = self.frames
            self.frames = []
        samples = {channel: [] for channel in self.request.channels}
        malformed = 0
        for frame in frames:
            data = frame.data
            if (frame.owner_generation != self.reader_generation or not data or
                data[0] not in (49, 50, 51) or len(data) != 1 + 2 * (data[0] - 48)):
                malformed += 1
                continue
            channel = (frame.arbitration_id & 0x78) >> 3
            for index in range(data[0] - 48):
                value = int.from_bytes(data[1+2*index:3+2*index], 'big', signed=True)
                samples[channel].append({'raw_pressure': value,
                    'sample_sequence': len(samples[channel]),
                    'robot_receive_time': frame.received_at,
                    'controller_timestamp': None,
                    'receive_sequence': frame.receive_sequence, 'packet_sample_index': index})
        channels = []
        for channel, rows in samples.items():
            values = [r['raw_pressure'] for r in rows]
            channels.append({'channel': channel, 'samples': rows,
                'baseline_statistics': {'count': len(values),
                    'mean': statistics.mean(values) if values else None,
                    'sample_standard_deviation': statistics.stdev(values) if len(values) > 1 else None,
                    'min': min(values) if values else None, 'max': max(values) if values else None}})
        return {'ok': True, 'source_return_completed': True,
            'kind': 'scientific_pressure_capture', 'units': 'controller_pressure_counts',
            'requested': self.request.model_dump(), 'started_at': self.started_at,
            'finished_at': self.clock(), 'reader_generation': self.reader_generation,
            'channels': channels, 'retained_frame_count': len(frames),
            'lost_frame_count': self.lost_frames, 'malformed_frame_count': malformed,
            'lost_sample_count': None, 'controller_timestamps_available': False,
            'sample_period_basis': 'authored_not_read_back',
            'anomaly_model': None, 'classification': None,
            'analysis_basis': 'descriptive_baseline_not_calibrated_anomaly_model',
            'physical_effect_verified': False}
