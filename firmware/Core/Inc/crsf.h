/******************************************************************************
 * Copyright (C) 2026 by Mohammed Kassem
 *
 * This software is provided for educational purposes.
 *
 *****************************************************************************/
/**
 * @file crsf.h
 * @brief CRSF (Crossfire) radio receiver parser
 *
 * This file contains the function declarations for the CRSF parser.
 * The bytes received from the radio receiver (see uart.h) are fed to
 * CRSF_process_byte() one at a time. Every time a full RC channels
 * packet has been decoded, the new channel values can be read with
 * CRSF_get_channel() or CRSF_get_channels().
 *
 * Channel values are 11 bits (0 to 2047).
 *
 * @author Mohammed Kassem
 * @date 10/09/2026
 *
 */

#ifndef CRSF_H
#define CRSF_H

#include <stdint.h>

/** Address byte that starts every CRSF packet sent to the flight controller */
#define CRSF_ADDRESS            0xC8U

/** Packet type of the RC channels packet (RC_CHANNELS_PACKED) */
#define CRSF_RC_CHANNELS        0x16U

/** Number of RC channels in one packet */
#define CRSF_RC_CHANNEL_COUNT   16U

/** Size of the packet buffer in bytes */
#define CRSF_MAX_PACKET_SIZE    64U

#define RC_TIMEOUT_MS   500U


/**
 * @brief Feeds one received byte to the CRSF parser
 *
 * Call this for every byte received from the radio receiver. The
 * function waits for the address byte, reads the length, collects the
 * rest of the packet and, if it is an RC channels packet, decodes the
 * 16 channels. Packets of any other type are ignored.
 *
 * @param byte The received byte
 * @return 1 if a new set of channel values has just been decoded,
 *         0 otherwise
 */
uint8_t CRSF_process_byte(uint8_t byte);

/**
 * @brief Gets the latest value of one channel
 *
 * @param channel Channel number (1 to 16)
 * @return Channel value (0 to 2047), or 0 if the channel number is invalid
 */
uint16_t CRSF_get_channel(uint8_t channel);

/**
 * @brief Gets the latest value of all 16 channels
 *
 * @param output Array of 16 values where the channels are stored
 *               (output[0] is channel 1)
 */
void CRSF_get_channels(uint16_t output[16]);

#endif /* CRSF_H */
