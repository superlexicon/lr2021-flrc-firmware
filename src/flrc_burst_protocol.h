/**
 * @file      flrc_burst_protocol.h
 *
 * @brief     CRC32 utility for hardware FLRC burst mode.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLRC_BURST_PROTOCOL_H
#define FLRC_BURST_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Calculate IEEE 802.3 CRC32 checksum over a byte buffer.
 */
uint32_t flrc_burst_calc_crc32(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* FLRC_BURST_PROTOCOL_H */
