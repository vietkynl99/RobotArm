#ifndef INC_PACKET_PACKER_H_
#define INC_PACKET_PACKER_H_

#include <string.h>
#include <cassert>
#include "Log.h"
#include "main.h"
#include "SpiTransferType.h"

class PacketPacker
{
public:
    static DataFrame create(uint8_t command, const uint8_t *data, size_t length);
    static void updateChecksum(DataFrame &frame);
    static bool verify(const DataFrame &frame);
};

#endif