"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class structFUSIONMAP(object):
    __slots__ = ["timestamp", "utmX", "utmY", "mHeading", "resolution", "rows", "cols", "center_col", "center_row", "cells"]

    def __init__(self):
        self.timestamp = 0
        self.utmX = 0.0
        self.utmY = 0.0
        self.mHeading = 0.0
        self.resolution = 0.0
        self.rows = 0
        self.cols = 0
        self.center_col = 0
        self.center_row = 0
        self.cells = [ "" for dim0 in range(501) ]

    def encode(self):
        buf = BytesIO()
        buf.write(structFUSIONMAP._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">qdddfhhhh", self.timestamp, self.utmX, self.utmY, self.mHeading, self.resolution, self.rows, self.cols, self.center_col, self.center_row))
        for i0 in range(501):
            buf.write(bytearray(self.cells[i0][:251]))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != structFUSIONMAP._get_packed_fingerprint():
            raise ValueError("Decode error")
        return structFUSIONMAP._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = structFUSIONMAP()
        self.timestamp, self.utmX, self.utmY, self.mHeading, self.resolution, self.rows, self.cols, self.center_col, self.center_row = struct.unpack(">qdddfhhhh", buf.read(44))
        self.cells = []
        for i0 in range(501):
            self.cells.append(buf.read(251))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if structFUSIONMAP in parents: return 0
        tmphash = (0x17ac376ea25ee2c9) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if structFUSIONMAP._packed_fingerprint is None:
            structFUSIONMAP._packed_fingerprint = struct.pack(">Q", structFUSIONMAP._get_hash_recursive([]))
        return structFUSIONMAP._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

