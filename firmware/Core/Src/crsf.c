/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file crsf.c
 * @brief CRSF (Crossfire) radio receiver parser
 *
 * This file parses the CRSF byte stream coming from the radio receiver.
 * Bytes are fed in one at a time. When a complete RC channels packet
 * (type 0x16) has been received, the 16 channels, each 11 bits wide and
 * packed one after the other, are unpacked and stored. The channel
 * values can then be read with CRSF_get_channel() or CRSF_get_channels().
 *
 * Packet layout:
 * byte 0      : address (CRSF_ADDRESS)
 * byte 1      : length (type + payload + CRC)
 * byte 2      : type (0x16 = RC_CHANNELS_PACKED)
 * bytes 3..24 : 22 bytes of packed channel data (16 channels x 11 bits)
 * byte 25     : CRC
 *
 * @author Mohammed Kassem
 * @date 10/09/2026
 *
 */

#include "crsf.h"


// Parser state
static uint8_t packet[CRSF_MAX_PACKET_SIZE];    // packet being received
static uint8_t index = 0;                       // next free position in packet
static uint8_t length = 0;                      // length byte of the packet

// Latest decoded channel values (11 bits each)
static uint16_t channels[CRSF_RC_CHANNEL_COUNT];


static void CRSF_decode_channels(void)
{
    // Channels start at byte 4
    uint8_t *data = &packet[3];

    for (int i = 0; i < CRSF_RC_CHANNEL_COUNT; i++)
    {
        // Each channel uses 11 bits
        // Find where this channel starts
        uint16_t bit_position = i * 11;

        // Find which byte contains the start of the channel
        uint8_t byte_position = bit_position / 8;

        // Find the bit location from the byte
        uint8_t bit_offset = bit_position % 8;

        // Read 3 bytes
        // 11 bit channel can cross into 3 bytes
        uint32_t value = data[byte_position] | ((uint32_t)data[byte_position + 1] << 8) | ((uint32_t)data[byte_position + 2] << 16);

        // Move the channel bits to the right and keep the lowest 11 bits
        channels[i] =
            (value >> bit_offset) & 0x07FF;
    }
}


uint8_t CRSF_process_byte(uint8_t byte)
{
    // Waiting for the start of a packet.
    if (index == 0)
    {
        if (byte != CRSF_ADDRESS)
        {
            return 0;
        }

        packet[index++] = byte;

        return 0;
    }


    // Second byte contains the packet length.
    if (index == 1)
    {
        length = byte;

        /*
         A CRSF packet cannot be shorter
         than 2 bytes after the length byte,
         or larger than 62 bytes
        */
        if (length < 2 || length > 62)
        {
            index = 0;

            return 0;
        }

        packet[index++] = byte;

        return 0;
    }


    // Store the rest of the packet.
    packet[index++] = byte;


    // Check if the complete packet has been received
    if (index == length + 2)
    {
        /*
         Check if this is an RC channel packet
         0x16 = RC_CHANNELS_PACKED
         length 24 = 1 type + 22 channel bytes + 1 CRC
         */
        if (packet[2] == CRSF_RC_CHANNELS &&
            length == 24)
        {
            CRSF_decode_channels();

            index = 0;

            return 1;
        }

        // Packet was valid, but it was a different CRSF packet type
        index = 0;

        return 0;
    }


    return 0;
}


uint16_t CRSF_get_channel(uint8_t channel)
{
    // Channel numbers are 1 to 16
    if (channel < 1 || channel > 16)
    {
        return 0;
    }

    return channels[channel - 1];
}


void CRSF_get_channels(uint16_t output[16])
{
    for (int i = 0; i < 16; i++)
    {
        output[i] = channels[i];
    }
}
