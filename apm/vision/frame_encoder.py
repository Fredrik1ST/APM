''' Frame encoders for converting BGRA camera frames to common image formats.'''

import cv2
import numpy as np
from apm.drivers.camera import CameraSnapshot

def encode_jpeg(snapshot: CameraSnapshot, quality: int = 80, scale: float = 1.0, display: bool = False) -> bytes:
    '''Convert a CameraSnapshot's image to JPEG bytes with specified quality and scale.'''
    bgr = snapshot.image[:, :, :3]  # Drop alpha channel (BGRA -> BGR) for OpenCV
    if scale != 1.0:
        bgr = cv2.resize(bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not ok:
        raise RuntimeError('JPEG encode failed')
    if display:
        cv2.imshow('Encoded JPEG', buf)
    return buf.tobytes()
