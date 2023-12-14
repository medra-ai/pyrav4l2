from __future__ import annotations
from fcntl import ioctl
import mmap
from select import select

from .device import Device
from .v4l2 import *

from dataclasses import dataclass

@dataclass
class Frame:
    data: bytes
    timestamp: float



class Stream:
    """
    Class that uses Device for capturing frames.
    When iterating over Stream object, it returns newly captured frame every iteration
    """

    def __init__(self, device: Device) -> None:
        """
        Parameters
        ----------
        device : Device
            Device that should be used for streaming

        Raises
        ------
        IOError
            If there is not enough memory for a buffer
        """

        self._context_level = 0
        self.device = device

    def start(self):
        self._open()

        ioctl(self.f_cam, VIDIOC_STREAMON,
              ctypes.c_int(V4L2_BUF_TYPE_VIDEO_CAPTURE))
        select((self.f_cam, ), (), ())

    def get_frame(self) -> Frame:
        buf = self.buffers[0][0]
        ioctl(self.f_cam, VIDIOC_DQBUF, buf)

        frame = self.buffers[buf.index][1][:buf.bytesused]
        timestamp = self.buffers[buf.index][0].timestamp.tv_usec * 1e-6 + self.buffers[buf.index][0].timestamp.tv_sec
        
        ioctl(self.f_cam, VIDIOC_QBUF, buf)
        return Frame(frame, timestamp)
    
    def close(self):
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
