/**
 * @file      flrc_burst_protocol.c
 *
 * @brief     CRC32 utility implementation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flrc_burst_protocol.h"

uint32_t flrc_burst_calc_crc32(const uint8_t *data, size_t len)
{
    uint32_t crc = 0xFFFFFFFFU;
    if (data == NULL || len == 0) return 0;

    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1U) {
                crc = (crc >> 1) ^ 0xEDB88320U;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}
