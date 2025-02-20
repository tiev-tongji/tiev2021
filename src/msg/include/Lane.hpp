/** THIS IS AN AUTOMATICALLY GENERATED FILE.
 *  DO NOT MODIFY BY HAND!!
 *
 *  Generated by zcm-gen
 **/

#include <zcm/zcm_coretypes.h>

#ifndef __Lane_hpp__
#define __Lane_hpp__

#include "LaneLine.hpp"
#include "LaneLine.hpp"


class Lane
{
    public:
        int32_t    lane_type;

        float      width;

        LaneLine   left_line;

        LaneLine   right_line;

    public:
        #if __cplusplus > 199711L /* if c++11 */
        static constexpr int8_t   kTypeNone = 0x00;
        static constexpr int8_t   kTypeStraight = 0x01;
        static constexpr int8_t   kTypeLeft = 0x02;
        static constexpr int8_t   kTypeStraightLeft = 0x03;
        static constexpr int8_t   kTypeRight = 0x04;
        static constexpr int8_t   kTypeStraightRight = 0x05;
        static constexpr int8_t   kTypeStraightLeftRight = 0x07;
        static constexpr int8_t   kTypeUTurn = 0x08;
        static constexpr int8_t   kTypeLeftRight = 0x06;
        static constexpr int8_t   kTypeLeftUTurn = 0x09;
        static constexpr int8_t   kTypeStraightUTurn = 0x0A;
        static constexpr int8_t   kTypeMerge = 0x0B;
        #else
        static const     int8_t   kTypeNone = 0x00;
        static const     int8_t   kTypeStraight = 0x01;
        static const     int8_t   kTypeLeft = 0x02;
        static const     int8_t   kTypeStraightLeft = 0x03;
        static const     int8_t   kTypeRight = 0x04;
        static const     int8_t   kTypeStraightRight = 0x05;
        static const     int8_t   kTypeStraightLeftRight = 0x07;
        static const     int8_t   kTypeUTurn = 0x08;
        static const     int8_t   kTypeLeftRight = 0x06;
        static const     int8_t   kTypeLeftUTurn = 0x09;
        static const     int8_t   kTypeStraightUTurn = 0x0A;
        static const     int8_t   kTypeMerge = 0x0B;
        #endif

    public:
        /**
         * Destructs a message properly if anything inherits from it
        */
        virtual ~Lane() {}

        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void* buf, uint32_t offset, uint32_t maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline uint32_t getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void* buf, uint32_t offset, uint32_t maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "Lane"
         */
        inline static const char* getTypeName();

        // ZCM support functions. Users should not call these
        inline int      _encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const;
        inline uint32_t _getEncodedSizeNoHash() const;
        inline int      _decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen);
        inline static uint64_t _computeHash(const __zcm_hash_ptr* p);
};

int Lane::encode(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;
    int64_t hash = (int64_t)getHash();

    thislen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int Lane::decode(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t Lane::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t Lane::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* Lane::getTypeName()
{
    return "Lane";
}

int Lane::_encodeNoHash(void* buf, uint32_t offset, uint32_t maxlen) const
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int32_t_encode_array(buf, offset + pos, maxlen - pos, &this->lane_type, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->width, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->left_line._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->right_line._encodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int Lane::_decodeNoHash(const void* buf, uint32_t offset, uint32_t maxlen)
{
    uint32_t pos = 0;
    int thislen;

    thislen = __int32_t_decode_array(buf, offset + pos, maxlen - pos, &this->lane_type, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->width, 1);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->left_line._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    thislen = this->right_line._decodeNoHash(buf, offset + pos, maxlen - pos);
    if(thislen < 0) return thislen; else pos += thislen;

    return pos;
}

uint32_t Lane::_getEncodedSizeNoHash() const
{
    uint32_t enc_size = 0;
    enc_size += __int32_t_encoded_array_size(NULL, 1);
    enc_size += __float_encoded_array_size(NULL, 1);
    enc_size += this->left_line._getEncodedSizeNoHash();
    enc_size += this->right_line._getEncodedSizeNoHash();
    return enc_size;
}

uint64_t Lane::_computeHash(const __zcm_hash_ptr* p)
{
    const __zcm_hash_ptr* fp;
    for(fp = p; fp != NULL; fp = fp->parent)
        if(fp->v == Lane::getHash)
            return 0;
    const __zcm_hash_ptr cp = { p, (void*)Lane::getHash };

    uint64_t hash = (uint64_t)0x2364f989fbf23e64LL +
         LaneLine::_computeHash(&cp) +
         LaneLine::_computeHash(&cp);

    return (hash<<1) + ((hash>>63)&1);
}

#endif
