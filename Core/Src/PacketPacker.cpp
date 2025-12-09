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

    update(frame);
    return frame;
}

void PacketPacker::update(DataFrame &frame)
{
    frame.crc = crc16_modbus((const uint8_t *)&frame, sizeof(DataFrame) - 2);
}

bool PacketPacker::verify(const DataFrame &frame)
{
    return frame.magicNumber == MAGIC_NUMBER && frame.crc == crc16_modbus((const uint8_t *)&frame, sizeof(DataFrame) - 2);
}

// CRC16-MODBUS, poly = 0xA001, init = 0xFFFF, LSB-first
uint16_t PacketPacker::crc16_modbus(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}