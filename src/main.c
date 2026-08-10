/**
 * @file      main.c
 *
 * @brief     LR2021 USB-UART <-> FLRC hardware-burst bridge firmware (Zephyr RTOS).
 *            Fire-and-forget FLRC burst mode with immediate transmissions,
 *            continuous burst RX, and application payload queuing with out-of-order
 *            packet reassembly.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <stdarg.h>
#include <stdbool.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/printk.h>
#include <errno.h>
#include <stdio.h>

#include "flrc_config.h"
#include "flrc_burst_protocol.h"

#include "smtc_rac.h"
#include "smtc_rac_api.h"
#include "smtc_sw_platform_helper.h"
#include "smtc_modem_hal.h"
#include "radio_planner_types.h"
#include <zephyr/usp/smtc_sw_platform_helper.h>
#include <zephyr/usp/smtc_zephyr_usp_api.h>

LOG_MODULE_REGISTER( lr2021_bridge, LOG_LEVEL_INF );

#define LR2021_NODE DT_ALIAS( lora_transceiver )
#if !DT_NODE_HAS_STATUS( LR2021_NODE, okay )
#error "LR2021 radio node not found."
#endif

/* GPIO control for Seeed XIAO nRF54L15 onboard RF switch power */
#if DT_NODE_EXISTS(DT_PATH(rfsw_pwr))
static const struct gpio_dt_spec rfsw_pwr_gpio = GPIO_DT_SPEC_GET(DT_PATH(rfsw_pwr), enable_gpios);
#endif
#if DT_NODE_EXISTS(DT_PATH(rfsw_ctl))
static const struct gpio_dt_spec rfsw_ctl_gpio = GPIO_DT_SPEC_GET(DT_PATH(rfsw_ctl), enable_gpios);
#endif

/* Default sync word (matches Semtech burst sample). */
static const uint8_t default_syncword_1[4] = { 0x90, 0x56, 0x34, 0x12 };
static const uint8_t default_syncword_2[4] = { 0x00, 0x00, 0x00, 0x00 };
static const uint8_t default_syncword_3[4] = { 0x00, 0x00, 0x00, 0x00 };

/* Forward declarations */
static int  usb_init( void );
static int  usb_tx( const uint8_t* data, uint16_t len );
static int  usb_rx_poll( void );
static void send_ready( void );
static void send_error( uint8_t code, const char* msg );
static void send_rx_packet( const uint8_t* payload, uint16_t len, int16_t rssi );
static void send_stats_packet( void );
static void send_time_packet( void );
static inline void send_debug_event( uint8_t event_id, uint32_t val ) { ( void ) event_id; ( void ) val; }
static void rac_post_callback( rp_status_t status );
static void start_burst_rx( void );
static void request_burst_tx( void );
static void execute_burst_tx( void );
static void prepare_next_tx_fragment( void );

/* Globals */
static const struct device* const cdc_dev = DEVICE_DT_GET( DT_CHOSEN( zephyr_console ) );

static uint8_t             g_radio_access_id;
static smtc_rac_context_t* g_rac_ctx;
static bool                g_radio_ok = false;

static struct
{
    uint32_t freq_hz;
    uint8_t  cr_host;
    int8_t   tx_pwr_dbm;
    uint32_t sync_word;
    uint8_t  role;          /* FLRC_ROLE_BOTH / TX_ONLY / RX_ONLY */
} g_cfg = {
    .freq_hz    = FLRC_DEFAULT_FREQ_HZ,
    .cr_host    = FLRC_DEFAULT_CODING_RATE,
    .tx_pwr_dbm = FLRC_DEFAULT_TX_PWR_DBM,
    .sync_word  = FLRC_DEFAULT_SYNC_WORD,
    .role       = FLRC_ROLE_BOTH,
};

/* Burst mode state */
typedef enum {
    BURST_RX,
    BURST_TX_ABORTING,
    BURST_TX
} burst_mode_t;
static volatile burst_mode_t g_burst_mode = BURST_RX;
static volatile bool         g_restart_rx_pending = false;

/* Transmit Window Configuration */
static uint32_t g_slot_offset_ms = 100;   /* Default slot offset within epoch */
static uint32_t g_slot_period_ms = 1000;  /* Default slot period (1 second) */

/* TX: Application pending buffer */
static uint8_t  g_pending_tx_buf[FLRC_BURST_MAX_TOTAL_PAYLOAD];
static uint32_t g_pending_tx_len = 0;
/* burst_id must be globally unique across all nodes so the multi-slot
 * reassembly doesn't mix packets from different senders into the same slot.
 * We offset each node's burst_id range by a per-node value derived from the
 * sync_word (which is shared but we hash the CONFIG serial_port path... no,
 * we don't have node identity at firmware level). Instead, we use a simple
 * time-based randomization: XOR the incrementing counter with a boot-time
 * random seed. This makes burst_ids from different nodes unlikely to collide. */
static uint16_t g_pending_burst_id = 0;
static uint16_t g_burst_id_offset  = 0; /* random offset set at boot */
static bool     g_pending_tx_valid = false;
static uint8_t  g_tx_repeat_count = 0;
/* CSMA/CA backoff: the superloop won't request a TX until this time (ms) has
 * passed. Set by the LBT busy handler to a random future time so nodes that
 * deferred simultaneously don't collide again on retry. */
static uint32_t g_tx_backoff_until_ms = 0;
/* One-time flag: TxOnly radio needs to abort the initial RX once. */
static bool     g_tx_only_rx_aborted = false;

/* Periodic stats emission: every 10s, emit a stats packet so the host
 * can track counter evolution (bursts_tx/rx, host_tx_dropped, etc.)
 * without polling. This is critical for diagnosing delivery degradation. */
#define STATS_EMIT_INTERVAL_MS  10000
static uint32_t g_last_stats_emit_ms = 0;

/* TX: On-air formatted burst buffer (array of 511-byte packets) */
#define MAX_AIR_PACKETS ((FLRC_BURST_MAX_TOTAL_PAYLOAD + FLRC_MAX_PACKET_CHUNK_SIZE - 1) / FLRC_MAX_PACKET_CHUNK_SIZE)
static uint8_t  g_tx_air_packets[MAX_AIR_PACKETS][FLRC_BURST_PKT_PAYLOAD];
static uint32_t g_tx_air_total_len = 0;
static uint32_t g_tx_bytes_sent = 0;
static uint16_t g_tx_fifo_len[2];
static uint16_t g_tx_next_buf_idx = 0;

/* RX: Concurrent multi-burst reassembly.
 *
 * In a multi-node scenario, the RX radio sees interleaved packets from
 * different senders (different burst_ids). A single reassembly buffer would
 * wipe the in-progress burst every time a new burst_id arrives. Instead, we
 * maintain a fixed pool of concurrent reassembly slots, each keyed by
 * burst_id, so overlapping bursts are reassembled independently. */
#define MAX_RX_PACKET_TRACK 128
#define MAX_CONCURRENT_BURSTS 4
#define BURST_REASM_PAYLOAD_MAX 2048   /* max payload per burst (consensus traffic) */
#define BURST_SLOT_TIMEOUT_MS 5000     /* evict stale partial bursts after 5s */

typedef struct {
    bool     active;
    uint16_t burst_id;
    uint16_t total_packets;
    uint32_t payload_len;
    uint32_t payload_crc;
    uint16_t received_count;
    bool     received_mask[MAX_RX_PACKET_TRACK];
    uint8_t  reassembly_buf[BURST_REASM_PAYLOAD_MAX];
    int16_t  last_rssi;
    uint32_t last_activity_ms;
} burst_reasm_slot_t;

static burst_reasm_slot_t g_rx_slots[MAX_CONCURRENT_BURSTS];
static int16_t  g_rx_last_rssi = 0;

/* Single packet RX buffer for RAC */
static uint8_t  g_rac_rx_pkt_buf[FLRC_BURST_PKT_PAYLOAD];

/* Stats */
static struct {
    uint32_t bursts_tx;
    uint32_t bursts_rx;
    uint32_t packets_tx;
    uint32_t packets_rx;
    uint32_t crc_errors;
    uint32_t host_tx_dropped;   /* deferred host-TX frames dropped (queue full / too big) */
} g_stats = { 0 };

/* ----------------------------------------------------------------------------
 * Deferred host USB transfer queue.
 *
 * The radio callback (rac_post_callback) runs inside smtc_rac_run_engine(),
 * which is polled in the main superloop. Doing the byte-polled USB CDC/UART
 * transfer (usb_tx) synchronously in the callback blocks the radio engine for
 * milliseconds per frame — the radio cannot re-arm RX (or start the next TX)
 * until the host handoff completes, which is the dominant source of TX/RX
 * turnaround latency on this half-duplex link.
 *
 * Instead, the callback copies the completed frame (header + payload + rssi,
 * or a READY/ERROR/STATS control frame) into this ring and returns immediately,
 * letting the radio state machine advance. The superloop drains the ring and
 * performs the (slow) USB transfer after smtc_rac_run_engine(), when the radio
 * is already settled. Single-threaded (callback + superloop both run in the
 * main thread), so the ring needs no lock.
 *
 * Slot size covers realistic consensus traffic (block proposals ~430B, receipts,
 * state diffs). The full 24KB burst is only used by the host benchmark scripts;
 * oversized frames are dropped (counted in g_stats.host_tx_dropped) rather than
 * blocking the radio.
 * ------------------------------------------------------------------------- */
#define HOST_TX_QUEUE_DEPTH 4
#define HOST_TX_SLOT_BYTES  2048
static uint8_t  g_host_tx_buf[HOST_TX_QUEUE_DEPTH][HOST_TX_SLOT_BYTES];
static uint16_t g_host_tx_len[HOST_TX_QUEUE_DEPTH];
static volatile uint8_t g_host_tx_head;   /* superloop consumes (advances on drain) */
static volatile uint8_t g_host_tx_tail;   /* callback produces (advances on queue) */

/* USB parser state */
typedef enum {
    RX_STATE_TAG,
    RX_STATE_TX_MAGIC,
    RX_STATE_LEN_LO,
    RX_STATE_LEN_HI,
    RX_STATE_PAYLOAD,
    RX_STATE_CONFIG,
    RX_STATE_SET_WINDOW,
} rx_state_t;

static volatile rx_state_t state     = RX_STATE_TAG;
static uint8_t              tag       = 0;
static uint16_t             frame_len = 0;
static uint16_t             have      = 0;
static uint8_t              config_body[FLRC_CONFIG_BODY_LEN];
static uint8_t              window_body[8];

static int usb_init( void )
{
    if( !device_is_ready( cdc_dev ) ) return -ENODEV;
    return 0;
}

static int usb_tx( const uint8_t* data, uint16_t len )
{
    if( !device_is_ready( cdc_dev ) ) return -ENODEV;
    for( uint16_t i = 0; i < len; i++ )
    {
        uart_poll_out( cdc_dev, data[i] );
        if( ( i & 0x7F ) == 0x7F )
        {
            k_busy_wait( 100 );
        }
    }
    return 0;
}

/*
 * Stage a frame for deferred USB transfer to the host. Up to three segments
 * (header / payload / trailer) are concatenated into one ring slot. Called
 * from rac_post_callback — never blocks on USB. If the frame won't fit a slot
 * or the ring is full, it is dropped and counted in g_stats.host_tx_dropped.
 *
 * The callback + superloop both run in the main thread (smtc_rac_run_engine
 * is polled inline), so head/tail need no lock; `volatile` only keeps the
 * compiler from caching them across k_yield().
 */
static void queue_host_tx( const uint8_t* seg1, uint16_t len1,
                           const uint8_t* seg2, uint16_t len2,
                           const uint8_t* seg3, uint16_t len3 )
{
    uint16_t total = (uint16_t)( len1 + len2 + len3 );
    if( total > HOST_TX_SLOT_BYTES )
    {
        g_stats.host_tx_dropped++;
        return;
    }

    uint8_t next = (uint8_t)( ( g_host_tx_tail + 1 ) % HOST_TX_QUEUE_DEPTH );
    if( next == g_host_tx_head )
    {
        /* Ring full — drop the oldest queued frame so the newest (most
         * time-relevant consensus data) is kept. */
        g_host_tx_head = (uint8_t)( ( g_host_tx_head + 1 ) % HOST_TX_QUEUE_DEPTH );
        g_stats.host_tx_dropped++;
    }

    uint8_t*  dst = g_host_tx_buf[g_host_tx_tail];
    uint16_t  off = 0;
    if( len1 ) { memcpy( dst + off, seg1, len1 ); off += len1; }
    if( len2 ) { memcpy( dst + off, seg2, len2 ); off += len2; }
    if( len3 ) { memcpy( dst + off, seg3, len3 ); off += len3; }
    g_host_tx_len[g_host_tx_tail] = off;
    g_host_tx_tail = next;
}

static void send_ready( void )
{
    uint8_t pkt[3];
    pkt[0] = FLRC_MSG_READY;
    pkt[1] = ( uint8_t )( FLRC_BURST_PKT_PAYLOAD & 0xFF );
    pkt[2] = ( uint8_t )( ( FLRC_BURST_PKT_PAYLOAD >> 8 ) & 0xFF );
    queue_host_tx( pkt, sizeof( pkt ), NULL, 0, NULL, 0 );
}

static void send_error( uint8_t code, const char* msg )
{
    uint8_t msg_len = msg ? ( uint8_t ) strlen( msg ) : 0;
    uint8_t header[3] = { FLRC_MSG_ERROR, code, msg_len };
    queue_host_tx( header, 3, ( const uint8_t* ) msg, msg_len, NULL, 0 );
}



static void send_rx_packet( const uint8_t* payload, uint16_t len, int16_t rssi )
{
    uint8_t header[3] = {
        FLRC_MSG_RX,
        ( uint8_t )( len & 0xFF ),
        ( uint8_t )( ( len >> 8 ) & 0xFF )
    };
    uint8_t rssi_buf[2] = {
        ( uint8_t )( rssi & 0xFF ),
        ( uint8_t )( ( rssi >> 8 ) & 0xFF )
    };
    queue_host_tx( header, 3, payload, ( len > 0 ) ? len : 0, rssi_buf, 2 );
}

static void send_stats_packet( void )
{
    uint8_t header[3] = { FLRC_MSG_STATS, 24, 0 };
    uint8_t stats_bytes[24];
    sys_put_le32( g_stats.bursts_tx, &stats_bytes[0] );
    sys_put_le32( g_stats.bursts_rx, &stats_bytes[4] );
    sys_put_le32( g_stats.packets_tx, &stats_bytes[8] );
    sys_put_le32( g_stats.packets_rx, &stats_bytes[12] );
    sys_put_le32( g_stats.crc_errors, &stats_bytes[16] );
    sys_put_le32( g_stats.host_tx_dropped, &stats_bytes[20] );
    queue_host_tx( header, 3, stats_bytes, sizeof( stats_bytes ), NULL, 0 );
}

static void send_time_packet( void )
{
    uint8_t pkt[5];
    pkt[0] = FLRC_MSG_TIME;
    sys_put_le32( smtc_modem_hal_get_time_in_ms( ), &pkt[1] );
    ( void ) usb_tx( pkt, sizeof( pkt ) );
}

/* ========================================================================== *
 * Hardware FLRC Burst — RX (always-on)
 * ========================================================================== */

static void start_burst_rx( void )
{
    g_rac_ctx->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    smtc_rac_radio_flrc_burst_params_t* p = &g_rac_ctx->radio_params.flrc_burst;
    p->is_tx                     = false;
    p->frequency_in_hz           = g_cfg.freq_hz;
    p->rx_frequency_offset_in_hz = 0;
    p->raw_bit_rate              = RAL_FLRC_RAW_BIT_RATE_2_600_MBPS;
    p->cr                        = RAL_FLRC_CR_1_1;
    p->pulse_shape               = RAL_FLRC_PULSE_SHAPE_BT_05;
    p->preamble_len              = RAL_FLRC_PREAMBLE_LENGTH_32_BITS;
    p->sync_word_len             = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    p->tx_syncword_index         = RAL_FLRC_TX_SYNCWORD_1;
    p->match_sync_word           = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    p->pld_is_fix                = false;
    p->crc_type                  = RAL_FLRC_CRC_2_BYTES;
    p->sync_word[0]              = default_syncword_1;
    p->sync_word[1]              = default_syncword_2;
    p->sync_word[2]              = default_syncword_3;
    p->crc_seed                  = 0xFFFFFFFF;
    p->crc_polynomial            = 0x755B;
    p->max_rx_size               = FLRC_BURST_PKT_PAYLOAD;
    p->burst_rx_size             = FLRC_BURST_MAX_TOTAL_PAYLOAD; /* 24576 — RAC needs full range for continuous RX */
    p->rx_burst_timeout_margin_ms = 60000; /* 60s — keeps RAC in continuous RX */
    p->min_interframe_delay_us = 300;

    g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer = g_rac_rx_pkt_buf;
    g_rac_ctx->smtc_rac_data_buffer_setup.size_of_rx_payload_buffer = sizeof( g_rac_rx_pkt_buf );

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    g_rac_ctx->scheduler_config.start_time_ms = 0;
    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;
    g_rac_ctx->scheduler_config.callback_pre_radio_transaction  = NULL;

    g_burst_mode = BURST_RX;

    smtc_rac_return_code_t rc = smtc_rac_submit_radio_transaction( g_radio_access_id );
    if( rc != SMTC_RAC_SUCCESS )
    {
        LOG_ERR( "start_burst_rx: submit rc=%d", ( int ) rc );
        send_debug_event( 10, ( uint32_t ) rc );
        g_restart_rx_pending = true;
    }
    else
    {
        send_debug_event( 1, 0 ); /* Debug event 1: RX started */
    }
}

/* ========================================================================== *
 * Hardware FLRC Burst — TX (Fire-and-forget windowed)
 * ========================================================================== */

static void request_burst_tx( void )
{
    send_debug_event( 2, g_pending_tx_len ); /* Debug event 2: TX requested */

    g_burst_mode = BURST_TX_ABORTING;
    smtc_rac_abort_radio_submit( g_radio_access_id );
}

static void execute_burst_tx( void )
{
    uint32_t payload_len = g_pending_tx_len;
    if( payload_len == 0 )
    {
        g_restart_rx_pending = true;
        return;
    }

    uint32_t payload_crc = flrc_burst_calc_crc32( g_pending_tx_buf, payload_len );
    uint16_t total_packets = ( payload_len + FLRC_MAX_PACKET_CHUNK_SIZE - 1 ) / FLRC_MAX_PACKET_CHUNK_SIZE;

    g_tx_air_total_len = total_packets * FLRC_BURST_PKT_PAYLOAD;

    for( uint16_t i = 0; i < total_packets; i++ )
    {
        uint32_t offset = i * FLRC_MAX_PACKET_CHUNK_SIZE;
        uint32_t chunk_len = ( payload_len - offset < FLRC_MAX_PACKET_CHUNK_SIZE ) ?
                             ( payload_len - offset ) : FLRC_MAX_PACKET_CHUNK_SIZE;

        flrc_packet_header_t hdr;
        hdr.magic             = sys_cpu_to_le16( FLRC_BURST_HEADER_MAGIC );
        hdr.burst_id          = sys_cpu_to_le16( g_pending_burst_id ^ g_burst_id_offset );
        hdr.packet_idx        = sys_cpu_to_le16( i );
        hdr.total_packets     = sys_cpu_to_le16( total_packets );
        hdr.total_payload_len = sys_cpu_to_le32( payload_len );
        hdr.payload_crc32     = sys_cpu_to_le32( payload_crc );

        uint8_t* pkt = g_tx_air_packets[i];
        memcpy( pkt, &hdr, sizeof( hdr ) );
        memcpy( pkt + sizeof( hdr ), g_pending_tx_buf + offset, chunk_len );
        if( sizeof( hdr ) + chunk_len < FLRC_BURST_PKT_PAYLOAD )
        {
            memset( pkt + sizeof( hdr ) + chunk_len, 0, FLRC_BURST_PKT_PAYLOAD - ( sizeof( hdr ) + chunk_len ) );
        }
    }

    LOG_INF( "Executing burst TX: %u bytes, %u packets, burst_id=%u", payload_len, total_packets, g_pending_burst_id );
    send_debug_event( 3, total_packets ); /* Debug event 3: TX submitting */

    g_rac_ctx->modulation_type = SMTC_RAC_MODULATION_FLRC_BURST;

    smtc_rac_radio_flrc_burst_params_t* p = &g_rac_ctx->radio_params.flrc_burst;
    p->is_tx             = true;
    p->frequency_in_hz   = g_cfg.freq_hz;
    p->tx_power_in_dbm   = g_cfg.tx_pwr_dbm;
    p->raw_bit_rate      = RAL_FLRC_RAW_BIT_RATE_2_600_MBPS;
    p->cr                = RAL_FLRC_CR_1_1;
    p->pulse_shape       = RAL_FLRC_PULSE_SHAPE_BT_05;
    p->preamble_len      = RAL_FLRC_PREAMBLE_LENGTH_32_BITS;
    p->sync_word_len     = RAL_FLRC_SYNCWORD_LENGTH_4_BYTES;
    p->tx_syncword_index = RAL_FLRC_TX_SYNCWORD_1;
    p->match_sync_word   = RAL_FLRC_RX_MATCH_SYNCWORD_1;
    p->pld_is_fix        = false;
    p->crc_type          = RAL_FLRC_CRC_2_BYTES;
    p->sync_word[0]      = default_syncword_1;
    p->sync_word[1]      = default_syncword_2;
    p->sync_word[2]      = default_syncword_3;
    p->crc_seed          = 0xFFFFFFFF;
    p->crc_polynomial    = 0x755B;
    p->min_interframe_delay_us = 300;
    p->max_rx_size       = FLRC_BURST_PKT_PAYLOAD;
    p->burst_tx_size     = g_tx_air_total_len;

    /* Prime double-buffer FIFO */
    g_tx_bytes_sent = 0;
    g_tx_next_buf_idx = 0;
    p->size_of_tx_fifo_payload_buffer[0] = FLRC_BURST_PKT_PAYLOAD;
    p->size_of_tx_fifo_payload_buffer[1] = FLRC_BURST_PKT_PAYLOAD;

    p->tx_fifo_payload_buffer[0]  = g_tx_air_packets[0];
    g_tx_fifo_len[0]              = FLRC_BURST_PKT_PAYLOAD;
    p->tx_fifo_payload_length[0]  = &g_tx_fifo_len[0];
    g_tx_bytes_sent += FLRC_BURST_PKT_PAYLOAD;
    g_tx_next_buf_idx = 1;

    if( g_tx_bytes_sent < g_tx_air_total_len )
    {
        p->tx_fifo_payload_buffer[1]  = g_tx_air_packets[1];
        g_tx_fifo_len[1]              = FLRC_BURST_PKT_PAYLOAD;
        p->tx_fifo_payload_length[1]  = &g_tx_fifo_len[1];
        g_tx_bytes_sent += FLRC_BURST_PKT_PAYLOAD;
        g_tx_next_buf_idx = 2;
    }

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    g_rac_ctx->scheduler_config.start_time_ms = 0;
    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;
    g_rac_ctx->scheduler_config.callback_pre_radio_transaction  = NULL;

    /* LBT enabled: carrier sense before TX prevents collisions between nodes.
     * In Both mode, the radio's own RX is on the same channel, but LBT checks
     * BEFORE aborting RX for TX — it sees the channel state at that instant.
     * The 5ms listen window catches peer bursts that started just before. */
    g_rac_ctx->lbt_context.lbt_enabled        = true;
    g_rac_ctx->lbt_context.listen_duration_ms = 5;
    g_rac_ctx->lbt_context.threshold_dbm      = -80;
    g_rac_ctx->lbt_context.bandwidth_hz       = 1200000;

    g_burst_mode = BURST_TX;
    g_stats.bursts_tx++;

    smtc_rac_return_code_t rc = smtc_rac_submit_radio_transaction( g_radio_access_id );
    if( rc != SMTC_RAC_SUCCESS )
    {
        LOG_ERR( "execute_burst_tx: submit rc=%d", ( int ) rc );
        send_debug_event( 11, ( uint32_t ) rc );
        g_restart_rx_pending = true;
    }
    else
    {
        send_debug_event( 4, 0 ); /* Debug event 4: TX submit OK */
    }
}

static void prepare_next_tx_fragment( void )
{
    if( g_tx_bytes_sent >= g_tx_air_total_len ) return;

    uint16_t pkt_idx = g_tx_next_buf_idx;
    uint16_t buf_idx = pkt_idx % 2;

    g_rac_ctx->radio_params.flrc_burst.tx_fifo_payload_buffer[buf_idx] = g_tx_air_packets[pkt_idx];
    g_tx_fifo_len[buf_idx] = FLRC_BURST_PKT_PAYLOAD;
    g_tx_bytes_sent += FLRC_BURST_PKT_PAYLOAD;
    g_tx_next_buf_idx++;
}

/* ========================================================================== *
 * RAC Post-Transaction Callback
 * ========================================================================== */

static void rac_post_callback( rp_status_t status )
{
    switch( status )
    {
    case RP_STATUS_TX_DONE:
        send_debug_event( 5, 0 ); /* Debug event 5: TX packet done */
        break;

    case RP_STATUS_RX_PACKET:
    {
        g_stats.packets_rx++;
        uint16_t rx_size = g_rac_ctx->smtc_rac_data_result.rx_size;
        int16_t  rssi    = g_rac_ctx->smtc_rac_data_result.rssi_result;
        g_rx_last_rssi   = rssi;

        send_debug_event( 6, rx_size ); /* Debug event 6: Single packet RX */

        uint8_t* rx_buf = g_rac_ctx->smtc_rac_data_buffer_setup.rx_payload_buffer;

        if( rx_size >= sizeof( flrc_packet_header_t ) )
        {
            flrc_packet_header_t hdr;
            memcpy( &hdr, rx_buf, sizeof( hdr ) );

            uint16_t magic             = sys_le16_to_cpu( hdr.magic );
            uint16_t burst_id          = sys_le16_to_cpu( hdr.burst_id );
            uint16_t packet_idx        = sys_le16_to_cpu( hdr.packet_idx );
            uint16_t total_packets     = sys_le16_to_cpu( hdr.total_packets );
            uint32_t total_payload_len = sys_le32_to_cpu( hdr.total_payload_len );
            uint32_t payload_crc32     = sys_le32_to_cpu( hdr.payload_crc32 );

            if( magic == FLRC_BURST_HEADER_MAGIC && total_packets > 0 && total_packets <= MAX_RX_PACKET_TRACK && total_payload_len <= BURST_REASM_PAYLOAD_MAX )
            {
                uint32_t now = smtc_modem_hal_get_time_in_ms( );

                /* Find or allocate a reassembly slot for this burst_id. */
                burst_reasm_slot_t* slot = NULL;
                int free_idx = -1;
                int oldest_idx = 0;
                uint32_t oldest_time = UINT32_MAX;

                for( int i = 0; i < MAX_CONCURRENT_BURSTS; i++ )
                {
                    if( g_rx_slots[i].active && g_rx_slots[i].burst_id == burst_id )
                    {
                        slot = &g_rx_slots[i];
                        break;
                    }
                    if( !g_rx_slots[i].active && free_idx < 0 )
                    {
                        free_idx = i;
                    }
                    /* Track oldest for eviction */
                    if( g_rx_slots[i].active && g_rx_slots[i].last_activity_ms < oldest_time )
                    {
                        oldest_time = g_rx_slots[i].last_activity_ms;
                        oldest_idx = i;
                    }
                }

                /* Evict stale slots (timeout-based cleanup) */
                for( int i = 0; i < MAX_CONCURRENT_BURSTS; i++ )
                {
                    if( g_rx_slots[i].active &&
                        ( now - g_rx_slots[i].last_activity_ms ) > BURST_SLOT_TIMEOUT_MS )
                    {
                        g_rx_slots[i].active = false;
                        if( free_idx < 0 ) free_idx = i;
                    }
                }

                if( slot == NULL )
                {
                    /* New burst_id — allocate a slot */
                    if( free_idx < 0 )
                    {
                        /* All slots full — evict the oldest */
                        free_idx = oldest_idx;
                        g_rx_slots[free_idx].active = false;
                    }
                    slot = &g_rx_slots[free_idx];
                    memset( slot, 0, sizeof( *slot ) );
                    slot->active        = true;
                    slot->burst_id      = burst_id;
                    slot->total_packets = total_packets;
                    slot->payload_len   = total_payload_len;
                    slot->payload_crc   = payload_crc32;
                    LOG_INF( "Started receiving burst_id=%u, pkts=%u, len=%u (slot %d)",
                             burst_id, total_packets, total_payload_len, free_idx );
                }

                slot->last_activity_ms = now;
                slot->last_rssi        = rssi;

                if( packet_idx < total_packets && !slot->received_mask[packet_idx] )
                {
                    slot->received_mask[packet_idx] = true;
                    slot->received_count++;

                    uint32_t offset = packet_idx * FLRC_MAX_PACKET_CHUNK_SIZE;
                    uint32_t chunk_len = ( total_payload_len - offset < FLRC_MAX_PACKET_CHUNK_SIZE ) ?
                                         ( total_payload_len - offset ) : FLRC_MAX_PACKET_CHUNK_SIZE;

                    if( offset + chunk_len <= sizeof( slot->reassembly_buf ) )
                    {
                        memcpy( &slot->reassembly_buf[offset],
                                rx_buf + sizeof( flrc_packet_header_t ), chunk_len );
                    }

                    /* Check if ALL packets received */
                    if( slot->received_count == slot->total_packets &&
                        slot->received_mask[0] &&
                        slot->received_mask[slot->total_packets - 1] )
                    {
                        uint32_t calc_crc = flrc_burst_calc_crc32(
                            slot->reassembly_buf, slot->payload_len );
                        if( calc_crc == slot->payload_crc )
                        {
                            g_stats.bursts_rx++;
                            LOG_INF( "Burst reassembly COMPLETE! burst_id=%u, %u bytes",
                                     burst_id, slot->payload_len );
                            send_rx_packet( slot->reassembly_buf,
                                            ( uint16_t ) slot->payload_len,
                                            slot->last_rssi );
                            smtc_rac_flrc_burst_rx_done( g_radio_access_id );
                        }
                        else
                        {
                            LOG_WRN( "CRC mismatch burst_id=%u: calc 0x%08x vs exp 0x%08x",
                                     burst_id, calc_crc, slot->payload_crc );
                        }
                        /* Free the slot regardless of CRC result */
                        slot->active = false;
                    }
                }
            }
            else
            {
                LOG_WRN("Invalid header magic 0x%04x or params", magic);
            }
        }
        break;
    }

    case RP_STATUS_REQUEST_NEXT_TX_PAYLOAD:
        g_stats.packets_tx++;
        prepare_next_tx_fragment( );
        break;

    case RP_STATUS_RX_CRC_ERROR:
        g_stats.crc_errors++;
        send_debug_event( 7, 0 ); /* Debug event 7: CRC error */
        break;

    case RP_STATUS_LBT_BUSY_CHANNEL:
        /* CSMA/CA: LBT detected RF energy — another node is transmitting.
         * Don't TX; apply a random backoff so nodes that deferred
         * simultaneously don't collide again on retry. The backoff grows
         * exponentially with consecutive deferrals (binary exponential
         * backoff, capped at 64ms) to break persistent contention. */
        g_tx_repeat_count++;
        if( g_tx_repeat_count > 10 )
        {
            /* Channel persistently busy — drop this frame to avoid
             * starving the radio with endless retries. */
            g_pending_tx_valid = false;
            g_pending_tx_len   = 0;
            g_tx_repeat_count  = 0;
        }
        else
        {
            /* Random backoff: 1..(2^N) ms, capped at 64ms.
             * g_tx_repeat_count starts at 1 after the first deferral. */
            uint32_t max_backoff = 1U << ( g_tx_repeat_count > 6 ? 6 : g_tx_repeat_count );
            uint32_t backoff_ms  = 1 + ( smtc_modem_hal_get_time_in_ms( ) % max_backoff );
            g_tx_backoff_until_ms = smtc_modem_hal_get_time_in_ms( ) + backoff_ms;
        }
        /* LBT detected busy channel: don't restart RX immediately.
         * Set g_burst_mode = BURST_RX so the TX trigger can retry after
         * backoff. Setting g_restart_rx_pending causes an infinite loop:
         * start_burst_rx() → TX trigger → abort → LBT → busy → restart...
         * The main loop's g_restart_rx_pending check also requires
         * g_burst_mode == BURST_RX, so setting burst_mode alone is enough
         * to allow TX retry without the RX restart loop. */
        g_burst_mode = BURST_RX;
        /* Do NOT set g_restart_rx_pending — it causes a tight loop. */
        break;

    case RP_STATUS_RADIO_UNLOCKED:
        if( g_burst_mode == BURST_TX_ABORTING && g_pending_tx_valid )
        {
            send_debug_event( 9, 0 ); /* Debug event 9: RX aborted for TX */
            execute_burst_tx( );
        }
        else if( g_burst_mode == BURST_TX )
        {
            LOG_INF( "TX burst complete" );
            send_debug_event( 8, 0 ); /* Debug event 8: TX complete */
            send_ready( );
            g_pending_tx_valid   = false;
            g_pending_tx_len     = 0;
            g_tx_repeat_count    = 0;
            g_burst_mode         = BURST_RX;
            g_restart_rx_pending = true;
        }
        else if( g_burst_mode == BURST_RX )
        {
            g_restart_rx_pending = true;
        }
        break;

    case RP_STATUS_TASK_ABORTED:
        send_debug_event( 9, 0 ); /* Debug event 9: Task aborted */
        if( g_burst_mode == BURST_TX_ABORTING && g_pending_tx_valid )
        {
            execute_burst_tx( );
        }
        else
        {
            g_pending_tx_valid = false;
            g_pending_tx_len   = 0;
            g_burst_mode       = BURST_RX;
            g_restart_rx_pending = true;
        }
        break;

    default:
        break;
    }
}

/* ========================================================================== *
 * Host command handling
 * ========================================================================== */

static void handle_config( const uint8_t* body )
{
    g_cfg.freq_hz    = sys_get_le32( &body[0] );
    g_cfg.cr_host    = body[8];
    g_cfg.tx_pwr_dbm = ( int8_t ) body[9];
    g_cfg.sync_word  = sys_get_le32( &body[10] );
    g_cfg.role       = body[14];

    /* After CONFIG, restart RX with the new freq/sync for ALL roles.
     * For TxOnly: the RAC needs an active RX transaction at all times so
     * that request_burst_tx() can abort it before TX. Without an active
     * transaction, smtc_rac_abort_radio_submit() has nothing to abort and
     * the RAC hangs. The actual reception is handled by the dedicated RX
     * radio on the second board — this RX is just to keep the RAC state
     * machine valid for TX abort/submit cycles. */
    g_restart_rx_pending = true;
}

static int usb_rx_poll( void )
{
    static uint32_t last_rx_time = 0;
    uint32_t now = k_uptime_get_32();

    /* Reset state machine if serial line has been idle for > 100ms */
    if( state != RX_STATE_TAG && ( now - last_rx_time > 100 ) )
    {
        state = RX_STATE_TAG;
        have = 0;
    }

    uint8_t b;
    while( uart_poll_in( cdc_dev, &b ) == 0 )
    {
        last_rx_time = k_uptime_get_32();
        switch( state )
        {
        case RX_STATE_TAG:
            tag = b;
            if( tag == FLRC_MSG_TX )
            {
                state = RX_STATE_TX_MAGIC;
                have = 0;
            }
            else if( tag == FLRC_MSG_SET_WINDOW )
            {
                state = RX_STATE_SET_WINDOW;
                have = 0;
            }
            else if( tag == FLRC_MSG_CONFIG )
            {
                state = RX_STATE_LEN_LO;
                have = 0;
            }
            else if( tag == FLRC_MSG_STATS )
            {
                /* Host requests a stats snapshot. Respond immediately
                 * via the deferred queue (non-blocking). */
                send_stats_packet( );
            }
            break;

        case RX_STATE_TX_MAGIC:
            if( b == 'X' )
            {
                state = RX_STATE_LEN_LO;
                have = 0;
            }
            else
            {
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_SET_WINDOW:
            window_body[have++] = b;
            if( have >= 8 )
            {
                g_slot_offset_ms = sys_get_le32( &window_body[0] );
                g_slot_period_ms = sys_get_le32( &window_body[4] );
                LOG_INF( "Window set: offset=%u ms, period=%u ms", g_slot_offset_ms, g_slot_period_ms );
                send_ready( );
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_LEN_LO:
            frame_len = b;
            state = RX_STATE_LEN_HI;
            break;

        case RX_STATE_LEN_HI:
            frame_len |= ( uint16_t )( b << 8 );
            have = 0;
            if( tag == FLRC_MSG_CONFIG )
            {
                if( frame_len != FLRC_CONFIG_BODY_LEN )
                {
                    send_error( FLRC_ERR_BAD_CONFIG, "bad len" );
                    state = RX_STATE_TAG;
                    break;
                }
                state = RX_STATE_CONFIG;
            }
            else /* FLRC_MSG_TX */
            {
                if( frame_len == 0 || frame_len > FLRC_BURST_MAX_TOTAL_PAYLOAD )
                {
                    send_error( FLRC_ERR_PAYLOAD_TOO_BIG, "bad tx len" );
                    state = RX_STATE_TAG;
                    break;
                }
                state = RX_STATE_PAYLOAD;
            }
            break;

        case RX_STATE_PAYLOAD:
            g_pending_tx_buf[have++] = b;
            if( have >= frame_len )
            {
                g_pending_tx_len = frame_len;
                g_pending_burst_id++;
                /* Apply per-node random offset so different nodes don't
                 * collide on the same burst_id. Without this, two nodes
                 * incrementing from 0 produce overlapping burst_ids, and
                 * the receiver's multi-slot reassembly mixes their packets
                 * into the same slot → CRC failure on both. */
                /* (actual offset applied at TX header build time) */
                g_pending_tx_valid = true;
                send_debug_event( 20, g_pending_tx_len ); /* Debug event 20: Payload enqueued */
                LOG_INF( "Host payload buffered: %u bytes, burst_id=%u", g_pending_tx_len, g_pending_burst_id );
                state = RX_STATE_TAG;
            }
            break;

        case RX_STATE_CONFIG:
            config_body[have++] = b;
            if( have >= FLRC_CONFIG_BODY_LEN )
            {
                handle_config( config_body );
                send_ready( );
                state = RX_STATE_TAG;
            }
            break;

        default:
            state = RX_STATE_TAG;
            break;
        }
    }
    return 0;
}

/* ========================================================================== *
 * Init + Main Loop
 * ========================================================================== */

static int init_radio( void )
{
    /* Enable Seeed XIAO nRF54L15 onboard RF switch power regulator */
#if DT_NODE_EXISTS(DT_PATH(rfsw_pwr))
    if (device_is_ready(rfsw_pwr_gpio.port)) {
        gpio_pin_configure_dt(&rfsw_pwr_gpio, GPIO_OUTPUT_ACTIVE);
        LOG_INF("Enabled RF switch power (gpio2 3 HIGH)");
    }
#endif
#if DT_NODE_EXISTS(DT_PATH(rfsw_ctl))
    if (device_is_ready(rfsw_ctl_gpio.port)) {
        gpio_pin_configure_dt(&rfsw_ctl_gpio, GPIO_OUTPUT_INACTIVE);
    }
#endif

    SMTC_SW_PLATFORM_INIT( );
    SMTC_SW_PLATFORM_VOID( smtc_rac_init( ) );

    g_radio_access_id = SMTC_SW_PLATFORM( smtc_rac_open_radio( RAC_HIGH_PRIORITY ) );
    if( g_radio_access_id == 0xFF ) return -1;

    g_rac_ctx = smtc_rac_get_context( g_radio_access_id );
    if( g_rac_ctx == NULL ) return -1;

    g_rac_ctx->scheduler_config.callback_post_radio_transaction = rac_post_callback;

    /* In TxOnly mode, don't start RX — the radio is purely a transmitter.
     * In Both and RxOnly modes, start continuous burst RX. */
    if( g_cfg.role != FLRC_ROLE_TX_ONLY )
    {
        start_burst_rx( );
    }
    return 0;
}

int main( void )
{
    /* Generate a per-boot random burst_id offset so different nodes don't
     * collide on the same burst_id values. Uses the hardware timer as entropy. */
    g_burst_id_offset = (uint16_t)( k_uptime_get_32() ^ ( k_uptime_get_32() >> 16 ) );
    if( g_burst_id_offset == 0 ) g_burst_id_offset = 1;

    LOG_INF( "LR2021 FLRC hardware-burst bridge booting (burst_id offset: 0x%04X)", g_burst_id_offset );

    if( usb_init( ) != 0 ) return 0;
    k_sleep( K_SECONDS( 1 ) );

    if( init_radio( ) == 0 )
    {
        g_radio_ok = true;
        g_last_stats_emit_ms = smtc_modem_hal_get_time_in_ms( );
        send_ready( );
    }
    else
    {
        g_radio_ok = false;
        send_error( FLRC_ERR_RADIO_INIT, "radio init failed" );
    }

    while( true )
    {
        ( void ) usb_rx_poll( );

        if( g_radio_ok )
        {
            /* RX rearm for ALL roles. Even TxOnly needs an active RX
             * transaction in the RAC so request_burst_tx() can abort it.
             * Without this, the RAC hangs after the first TX completes. */
            if( g_restart_rx_pending && g_burst_mode == BURST_RX )
            {
                g_restart_rx_pending = false;
                start_burst_rx( );
            }

            /* TX trigger — only for roles that transmit (Both, TxOnly).
             * RxOnly never transmits.
             * For TxOnly: the initial RX (on default freq) is automatically
             * aborted by request_burst_tx() when the first TX is queued. */
            if( g_cfg.role != FLRC_ROLE_RX_ONLY &&
                g_pending_tx_valid && g_burst_mode == BURST_RX &&
                smtc_modem_hal_get_time_in_ms( ) >= g_tx_backoff_until_ms )
            {
                request_burst_tx( );
            }

            ( void ) smtc_rac_run_engine( );

            /* Emit periodic stats every 10s so the host can track counter
             * evolution (especially host_tx_dropped) without polling. */
            {
                uint32_t now_ms = smtc_modem_hal_get_time_in_ms( );
                if( now_ms - g_last_stats_emit_ms >= STATS_EMIT_INTERVAL_MS )
                {
                    g_last_stats_emit_ms = now_ms;
                    send_stats_packet( );
                }
            }

            /* Drain ONE deferred host-TX frame per loop iteration, AFTER the
             * radio engine has run. This performs the (slow) byte-polled USB
             * transfer here in the superloop — never inside rac_post_callback
             * — so the radio is never blocked waiting on USB handoff. Draining
             * one frame at a time interleaves USB I/O with radio-engine polls,
             * bounding the gap between engine runs to a single frame's TX time
             * (~1ms for a 430-byte proposal) instead of the whole queue. */
            if( g_host_tx_head != g_host_tx_tail )
            {
                ( void ) usb_tx( g_host_tx_buf[g_host_tx_head], g_host_tx_len[g_host_tx_head] );
                g_host_tx_head = (uint8_t)( ( g_host_tx_head + 1 ) % HOST_TX_QUEUE_DEPTH );
            }
        }

        k_yield( );
    }

    return 0;
}
