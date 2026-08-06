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
} g_cfg = {
    .freq_hz    = FLRC_DEFAULT_FREQ_HZ,
    .cr_host    = FLRC_DEFAULT_CODING_RATE,
    .tx_pwr_dbm = FLRC_DEFAULT_TX_PWR_DBM,
    .sync_word  = FLRC_DEFAULT_SYNC_WORD,
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
static uint16_t g_pending_burst_id = 0;
static bool     g_pending_tx_valid = false;
static uint8_t  g_tx_repeat_count = 0;

/* TX: On-air formatted burst buffer (array of 511-byte packets) */
#define MAX_AIR_PACKETS ((FLRC_BURST_MAX_TOTAL_PAYLOAD + FLRC_MAX_PACKET_CHUNK_SIZE - 1) / FLRC_MAX_PACKET_CHUNK_SIZE)
static uint8_t  g_tx_air_packets[MAX_AIR_PACKETS][FLRC_BURST_PKT_PAYLOAD];
static uint32_t g_tx_air_total_len = 0;
static uint32_t g_tx_bytes_sent = 0;
static uint16_t g_tx_fifo_len[2];
static uint16_t g_tx_next_buf_idx = 0;

/* RX: Out-of-order reassembly buffer */
#define MAX_RX_PACKET_TRACK 128
static uint8_t  g_rx_reassembly_buf[FLRC_BURST_MAX_TOTAL_PAYLOAD];
static bool     g_rx_received_mask[MAX_RX_PACKET_TRACK];
static uint16_t g_rx_received_count = 0;
static uint16_t g_rx_total_packets = 0;
static uint32_t g_rx_payload_len = 0;
static uint32_t g_rx_payload_crc = 0;
static uint16_t g_rx_current_burst_id = 0xFFFF;
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
} g_stats = { 0 };

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

static void send_ready( void )
{
    uint8_t pkt[3];
    pkt[0] = FLRC_MSG_READY;
    pkt[1] = ( uint8_t )( FLRC_BURST_PKT_PAYLOAD & 0xFF );
    pkt[2] = ( uint8_t )( ( FLRC_BURST_PKT_PAYLOAD >> 8 ) & 0xFF );
    ( void ) usb_tx( pkt, sizeof( pkt ) );
}

static void send_error( uint8_t code, const char* msg )
{
    uint8_t msg_len = msg ? ( uint8_t ) strlen( msg ) : 0;
    uint8_t header[3] = { FLRC_MSG_ERROR, code, msg_len };
    ( void ) usb_tx( header, 3 );
    if( msg_len > 0 ) ( void ) usb_tx( ( const uint8_t* ) msg, msg_len );
}



static void send_rx_packet( const uint8_t* payload, uint16_t len, int16_t rssi )
{
    uint8_t header[3] = {
        FLRC_MSG_RX,
        ( uint8_t )( len & 0xFF ),
        ( uint8_t )( ( len >> 8 ) & 0xFF )
    };
    ( void ) usb_tx( header, 3 );
    if( len > 0 && payload ) ( void ) usb_tx( payload, len );
    uint8_t rssi_buf[2] = {
        ( uint8_t )( rssi & 0xFF ),
        ( uint8_t )( ( rssi >> 8 ) & 0xFF )
    };
    ( void ) usb_tx( rssi_buf, 2 );
}

static void send_stats_packet( void )
{
    uint8_t header[3] = { FLRC_MSG_STATS, 20, 0 };
    ( void ) usb_tx( header, 3 );
    uint8_t stats_bytes[20];
    sys_put_le32( g_stats.bursts_tx, &stats_bytes[0] );
    sys_put_le32( g_stats.bursts_rx, &stats_bytes[4] );
    sys_put_le32( g_stats.packets_tx, &stats_bytes[8] );
    sys_put_le32( g_stats.packets_rx, &stats_bytes[12] );
    sys_put_le32( g_stats.crc_errors, &stats_bytes[16] );
    ( void ) usb_tx( stats_bytes, sizeof( stats_bytes ) );
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
    p->cr                        = RAL_FLRC_CR_3_4;
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
    p->burst_rx_size             = FLRC_BURST_MAX_TOTAL_PAYLOAD;
    p->rx_burst_timeout_margin_ms = 60000; /* 60-second RX window margin */
    p->min_interframe_delay_us = 1500;

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
        hdr.burst_id          = sys_cpu_to_le16( g_pending_burst_id );
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
    p->cr                = RAL_FLRC_CR_3_4;
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
    p->min_interframe_delay_us = 1500;
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

            if( magic == FLRC_BURST_HEADER_MAGIC && total_packets > 0 && total_packets <= MAX_RX_PACKET_TRACK && total_payload_len <= FLRC_BURST_MAX_TOTAL_PAYLOAD )
            {
                /* Reset reassembly if new burst_id detected */
                if( burst_id != g_rx_current_burst_id )
                {
                    g_rx_current_burst_id = burst_id;
                    g_rx_total_packets    = total_packets;
                    g_rx_payload_len      = total_payload_len;
                    g_rx_payload_crc      = payload_crc32;
                    g_rx_received_count   = 0;
                    memset( g_rx_received_mask, 0, sizeof( g_rx_received_mask ) );
                    memset( g_rx_reassembly_buf, 0, sizeof( g_rx_reassembly_buf ) );
                    LOG_INF("Started receiving burst_id=%u, total_packets=%u, len=%u", burst_id, total_packets, total_payload_len);
                }

                if( packet_idx < total_packets && !g_rx_received_mask[packet_idx] )
                {
                    g_rx_received_mask[packet_idx] = true;
                    g_rx_received_count++;

                    uint32_t offset = packet_idx * FLRC_MAX_PACKET_CHUNK_SIZE;
                    uint32_t chunk_len = ( total_payload_len - offset < FLRC_MAX_PACKET_CHUNK_SIZE ) ?
                                         ( total_payload_len - offset ) : FLRC_MAX_PACKET_CHUNK_SIZE;

                    if( offset + chunk_len <= sizeof( g_rx_reassembly_buf ) )
                    {
                        memcpy( &g_rx_reassembly_buf[offset], rx_buf + sizeof( flrc_packet_header_t ), chunk_len );
                    }

                    /* Check if ALL packets from 0 to total_packets - 1 are received */
                    if( g_rx_received_count == g_rx_total_packets &&
                        g_rx_received_mask[0] &&
                        g_rx_received_mask[g_rx_total_packets - 1] )
                    {
                        uint32_t calc_crc = flrc_burst_calc_crc32( g_rx_reassembly_buf, g_rx_payload_len );
                        if( calc_crc == g_rx_payload_crc )
                        {
                            g_stats.bursts_rx++;
                            LOG_INF("Burst reassembly COMPLETE! Sending %u bytes to host", g_rx_payload_len);
                            send_rx_packet( g_rx_reassembly_buf, ( uint16_t ) g_rx_payload_len, g_rx_last_rssi );
                            smtc_rac_flrc_burst_rx_done( g_radio_access_id );
                        }
                        else
                        {
                            LOG_WRN("CRC mismatch on reassembled burst: calc 0x%08x vs expected 0x%08x", calc_crc, g_rx_payload_crc);
                        }
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

    start_burst_rx( );
    return 0;
}

int main( void )
{
    LOG_INF( "LR2021 FLRC hardware-burst bridge booting" );

    if( usb_init( ) != 0 ) return 0;
    k_sleep( K_SECONDS( 1 ) );

    if( init_radio( ) == 0 )
    {
        g_radio_ok = true;
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
            if( g_restart_rx_pending && g_burst_mode == BURST_RX )
            {
                g_restart_rx_pending = false;
                start_burst_rx( );
            }

            /* Immediate TX trigger once host payload buffering completes */
            if( g_pending_tx_valid && g_burst_mode == BURST_RX )
            {
                request_burst_tx( );
            }

            ( void ) smtc_rac_run_engine( );
        }

        k_yield( );
    }

    return 0;
}
