#include "PacketPacker.h"

DataFrame PacketPacker::create(uint8_t command, const uint8_t *data, size_t length)
{
    DataFrame frame;
    frame.magicNumber = MAGIC_NUMBER;
    frame.command = command;
    if (data && length > 0)
    {
        if (length > SPI_DATA_SIZE)
        {
            length = SPI_DATA_SIZE;
        }
        memcpy(frame.data.data, data, length);
    }

    updateChecksum(frame);
    return frame;
}

void PacketPacker::updateChecksum(DataFrame &frame)
{
    frame.checksum = 0;
    const uint8_t *bytePtr = reinterpret_cast<const uint8_t *>(&frame);
    for (size_t i = 0; i < sizeof(frame) - 1; ++i)
    {
        frame.checksum += bytePtr[i];
    }
}

bool PacketPacker::verify(const DataFrame &frame)
{
    if (frame.magicNumber != MAGIC_NUMBER)
    {
        return false;
    }

    uint8_t checksum = 0;
    const uint8_t *bytePtr = reinterpret_cast<const uint8_t *>(&frame);
    for (size_t i = 0; i < sizeof(frame) - 1; ++i)
    {
        checksum += bytePtr[i];
    }
    return checksum == frame.checksum;
}
