from __future__ import annotations
from fcntl import ioctl
import mmap
from select import select

from .device import Device
from .v4l2 import *

from dataclasses import dataclass

@dataclass
class Frame:
    """Class to store the information of a frame"""
    data: bytes
    timestamp: float


class Stream:
    """Class that uses Device for capturing frames.

    Args:
        device (Device): Device to grab frames from
    """

    def __init__(self, device: Device) -> None:
        self._context_level = 0
        self.device = device

    def start(self):
        """Start the stream"""

        self._open()

        ioctl(self.f_cam, VIDIOC_STREAMON, ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
        select((self.f_cam, ), (), ())

    def get_frame(self) -> Frame:
        """Get a frame from the buffer
        
        Returns:
            Frame: Dataclass containing the bytes information of a frame, as well as its timestamp
        """

        buf = self.buffers[0][0]
        ioctl(self.f_cam, VIDIOC_DQBUF, buf)

        frame = self.buffers[buf.index][1][:buf.bytesused]
        timestamp = self.buffers[buf.index][0].timestamp.tv_usec * 1e-6 + self.buffers[buf.index][0].timestamp.tv_sec
        
        ioctl(self.f_cam, VIDIOC_QBUF, buf)
        return Frame(frame, timestamp)
    
    def close(self):
        """Close the stream"""
        self._stop()

    def _open(self):
        self.f_cam = open(self.device.path, "rb+", buffering=0)

        req = v4l2_requestbuffers()
        req.count = 4
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
        req.memory = V4L2_MEMORY_MMAP
        ioctl(self.f_cam, VIDIOC_REQBUFS, req)

        if req.count == 0:
            raise IOError("Not enough buffer memory")

        self.buffers = []
        for i in range(req.count):
            buf = v4l2_buffer()
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            buf.memory = V4L2_MEMORY_MMAP
            buf.index = i
            ioctl(self.f_cam, VIDIOC_QUERYBUF, buf)

            buffer = mmap.mmap(self.f_cam.fileno(),
                               length=buf.length,
                               flags=mmap.MAP_SHARED,
                               prot=mmap.PROT_READ,
                               offset=buf.m.offset)
            ioctl(self.f_cam, VIDIOC_QBUF, buf)

            self.buffers.append((buf, buffer))

    def _stop(self) -> None:
        ioctl(self.f_cam, VIDIOC_STREAMOFF,
              ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
        for buffer in self.buffers:
            buffer[1].close()
        self.f_cam.close()
