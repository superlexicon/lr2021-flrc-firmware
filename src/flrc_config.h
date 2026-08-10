/**
 * @file      flrc_config.h
 *
 * @brief     Configuration constants for LR2021 FLRC hardware-burst bridge.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FLRC_CONFIG_H
#define FLRC_CONFIG_H

#include <stdint.h>

/* Default Radio Settings */
#define FLRC_DEFAULT_FREQ_HZ       867100000U  /* 867.1 MHz Sub-GHz RF path */
#define FLRC_DEFAULT_CODING_RATE   1           /* CR_3_4 */
#define FLRC_DEFAULT_TX_PWR_DBM    0           /* 0 dBm output power (prevents desktop LNA saturation) */
#define FLRC_DEFAULT_SYNC_WORD     0x12345690U /* Default sync word */

/* Maximum application payload and on-air packet sizing */
#define FLRC_BURST_MAX_TOTAL_PAYLOAD 24576
#define FLRC_BURST_PKT_PAYLOAD       252
#define FLRC_BURST_HEADER_MAGIC      0x4642 /* 'FB' */

/* Host Protocol Messages */
#define FLRC_MSG_READY      'G'
#define FLRC_MSG_TX         'T'
#define FLRC_MSG_RX         'R'
#define FLRC_MSG_CONFIG     'C'
#define FLRC_MSG_SET_WINDOW 'W'
#define FLRC_MSG_STATS      'S'
#define FLRC_MSG_TIME       'M'
#define FLRC_MSG_ERROR      'E'

/* Error codes */
#define FLRC_ERR_RADIO_INIT       0x01
#define FLRC_ERR_PAYLOAD_TOO_BIG  0x02
#define FLRC_ERR_BAD_CONFIG       0x03

#define FLRC_CONFIG_BODY_LEN      15  /* freq(4)+bw(4)+cr(1)+txpwr(1)+sync(4)+role(1) */

/* Radio role (byte 14 of CONFIG body) */
#define FLRC_ROLE_BOTH      0   /* TX + RX — single-radio mode (default) */
#define FLRC_ROLE_TX_ONLY   1   /* TX only — skips reader task / RX */
#define FLRC_ROLE_RX_ONLY   2   /* RX only — skips discovery TX / outbound TX */

/* On-air packet header (16 bytes) */
typedef struct __attribute__((packed)) {
    uint16_t magic;             /* Magic 'FB' */
    uint16_t burst_id;          /* Incrementing burst ID */
    uint16_t packet_idx;        /* 0-indexed packet sequence number */
    uint16_t total_packets;     /* Total number of packets in this burst */
    uint32_t total_payload_len; /* Total burst application payload length */
    uint32_t payload_crc32;     /* Full payload CRC32 checksum */
} flrc_packet_header_t;

#define FLRC_MAX_PACKET_CHUNK_SIZE (FLRC_BURST_PKT_PAYLOAD - sizeof(flrc_packet_header_t))

#endif /* FLRC_CONFIG_H */
