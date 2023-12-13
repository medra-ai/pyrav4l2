from typing import Sequence, Type, cast
from pyrav4l2.device import Device, Control
from pyrav4l2.stream import Stream, Frame

import time
import numpy as np

import cv2 as cv

dev = Device("/dev/video0")
available_controls: Sequence[Type[Control]] = dev.controls

for control in available_controls:
    print(control.name)

color_format, frame_size = dev.get_format()
frame_intervals = dev.get_available_frame_intervals(color_format, frame_size)

stream = Stream(dev)
start_time = None
end_time = 0
for i, frame in enumerate(stream):
    frame = cast(Frame, frame)
    print("\n")
    call_time = time.monotonic()
    if start_time is None:
        start_time = call_time
    print(f"Frame {i} calltime: {call_time}")
    print(f"Frame {i}: {len(frame.data)} bytes")
    print(f"Frame {i} timestamp: {frame.timestamp}")
    image = cv.imdecode(np.frombuffer(frame.data, dtype=np.uint8), cv.IMREAD_COLOR)
    print(f"{call_time = }, {frame.timestamp = }")

    if call_time > end_time:
        end_time = call_time
    if i >= 10:
        print(f"Took {end_time - start_time} seconds")
        break
