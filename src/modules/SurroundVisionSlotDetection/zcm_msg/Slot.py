"""ZCM type definitions
This file automatically generated by zcm.
DO NOT MODIFY BY HAND!!!!
"""

try:
    import cStringIO.StringIO as BytesIO
except ImportError:
    from io import BytesIO
import struct

from zcm_msg.SlotPoint import SlotPoint

class Slot(object):
    __slots__ = ["front_left", "front_right", "rear_left", "rear_right"]

    def __init__(self):
        self.front_left = SlotPoint()
        self.front_right = SlotPoint()
        self.rear_left = SlotPoint()
        self.rear_right = SlotPoint()

    def encode(self):
        buf = BytesIO()
        buf.write(Slot._get_packed_fingerprint())
        self._encode_one(buf)
        return buf.getvalue()

    def _encode_one(self, buf):
        assert self.front_left._get_packed_fingerprint() == SlotPoint._get_packed_fingerprint()
        self.front_left._encode_one(buf)
        assert self.front_right._get_packed_fingerprint() == SlotPoint._get_packed_fingerprint()
        self.front_right._encode_one(buf)
        assert self.rear_left._get_packed_fingerprint() == SlotPoint._get_packed_fingerprint()
        self.rear_left._encode_one(buf)
        assert self.rear_right._get_packed_fingerprint() == SlotPoint._get_packed_fingerprint()
        self.rear_right._encode_one(buf)

    def decode(data):
        if hasattr(data, 'read'):
            buf = data
        else:
            buf = BytesIO(data)
        if buf.read(8) != Slot._get_packed_fingerprint():
            raise ValueError("Decode error")
        return Slot._decode_one(buf)
    decode = staticmethod(decode)

    def _decode_one(buf):
        self = Slot()
        self.front_left = SlotPoint._decode_one(buf)
        self.front_right = SlotPoint._decode_one(buf)
        self.rear_left = SlotPoint._decode_one(buf)
        self.rear_right = SlotPoint._decode_one(buf)
        return self
    _decode_one = staticmethod(_decode_one)

    _hash = None
    def _get_hash_recursive(parents):
        if Slot in parents: return 0
        newparents = parents + [Slot]
        tmphash = (0x536c6f9868acf008+ SlotPoint._get_hash_recursive(newparents)+ SlotPoint._get_hash_recursive(newparents)+ SlotPoint._get_hash_recursive(newparents)+ SlotPoint._get_hash_recursive(newparents)) & 0xffffffffffffffff
        tmphash  = (((tmphash<<1)&0xffffffffffffffff)  + ((tmphash>>63)&0x1)) & 0xffffffffffffffff
        return tmphash
    _get_hash_recursive = staticmethod(_get_hash_recursive)
    _packed_fingerprint = None

    def _get_packed_fingerprint():
        if Slot._packed_fingerprint is None:
            Slot._packed_fingerprint = struct.pack(">Q", Slot._get_hash_recursive([]))
        return Slot._packed_fingerprint
    _get_packed_fingerprint = staticmethod(_get_packed_fingerprint)

