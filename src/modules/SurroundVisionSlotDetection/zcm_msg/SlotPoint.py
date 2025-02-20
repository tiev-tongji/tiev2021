"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

class SlotPoint(object):
    __slots__ = ["x", "y"]

    def __init__(self):
        self.x = 0.0
        self.y = 0.0

    def encode(self):
        buf = BytesIO()
        buf.write(SlotPoint._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        buf.write(struct.pack(">dd", self.x, self.y))

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != SlotPoint._get_packed_fingerprint():
            raise ValueError("Decode error")
        return SlotPoint._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = SlotPoint()
        self.x, self.y = struct.unpack(">dd", buf.read(16))
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if SlotPoint in parents: return 0
        tmphash = (0xc421d470f254966b) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if SlotPoint._packed_fingerprint is None:
            SlotPoint._packed_fingerprint = struct.pack(">Q", SlotPoint._get_hash_recursive([]))
        return SlotPoint._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

