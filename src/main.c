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
#include <lr20xx_radio_flrc.h>
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
/* Debug event frame: 'D' + len(6) + event_id:u8 + val:LE32. Host logs
 * these as FW-DIAG lines. Event registry:
 *   1 RX started | 2 TX requested | 3 TX submitting | 4 TX submit ok
 *   5 TX packet done | 6 single packet RX | 7 CRC error | 8 TX complete
 *   9 RX aborted for TX | 10/11 submit failures | 20 payload staged
 *  22 TX-complete -> RX-restart gap (ms)
 *  23 run_engine stall >20ms (ms) — usb_tx drain cost
 *  24 hw-received-packets minus app-delivered (per stats interval)
 *  25 LBT-busy occurrence */
static void queue_host_tx( const uint8_t* p1, uint16_t l1, const uint8_t* p2, uint16_t l2,
                           const uint8_t* p3, uint16_t l3 );
static void send_debug_event( uint8_t event_id, uint32_t val )
{
    /* Only the low-frequency diagnostics (22-25) go on the wire: the
     * legacy per-packet events (1-11, 20) fire per RX packet and flood
     * the 3-deep host ring, dropping real traffic. */
    if( event_id < 22 )
    {
        return;
    }
    uint8_t frame[3 + 6] = { 'D', 6, 0, 0, 0, 0, 0, 0, 0 };
    frame[3] = event_id;
    sys_put_le32( val, &frame[4] );
    /* frame[8] = 0 (pad, already zeroed) */
    queue_host_tx( frame, sizeof( frame ), NULL, 0, NULL, 0 );
}

static void send_stats_packet( void );
static void send_time_packet( void );
static void send_debug_event( uint8_t event_id, uint32_t val );
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

/* TX watchdog: if a TX burst doesn't complete within this timeout, abort
 * and restart. The RAC engine's TX transaction can hang after sustained
 * operation (~115s). Without a watchdog, g_burst_mode stays BURST_TX
 * forever, blocking all further TX and RX. */
#define TX_WATCHDOG_TIMEOUT_MS  5000
static volatile uint32_t g_tx_start_time_ms = 0;

/* Transmit Window Configuration */
static uint32_t g_slot_offset_ms = 100;   /* Default slot offset within epoch */
static uint32_t g_slot_period_ms = 1000;  /* Default slot period (1 second) */

/* TX: Application pending buffer — the buffer the radio machinery reads
 * (execute_burst_tx copies it into the air-packet staging; the RAC
 * callbacks clear its valid flag on completion/drop). Never written by
 * the parser while a burst may be executing. */
static uint8_t  g_pending_tx_buf[FLRC_BURST_MAX_TOTAL_PAYLOAD];
static uint32_t g_pending_tx_len = 0;

/* TX: staging slot — the parser writes here. The superloop promotes a
 * completed staging frame into the pending buffer when the radio is free,
 * giving a 2-deep queue (one airing, one queued). The host's windowed
 * pipeline (max 2 frames outstanding) matches this depth; a frame that
 * completes while staging is still occupied overwrites it (counted). */
static uint8_t  g_staging_tx_buf[FLRC_BURST_MAX_TOTAL_PAYLOAD];
static uint32_t g_staging_tx_len = 0;
static bool     g_staging_tx_valid = false;
static uint32_t g_staging_overwrites = 0;
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
/* Timestamp of the last TX-complete (for the event-22 restart-gap measure). */
static uint32_t g_last_tx_complete_ms = 0;
/* Event-24 baselines: last hardware/app RX packet counts. */
static uint32_t g_hw_rx_last = 0;
static uint32_t g_app_rx_last = 0;
/* Engine-loop instrumentation: superloop iteration start time and worst
 * observed run_engine stall (event 23). */
static uint32_t g_loop_iter_start_ms = 0;
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
/* 3 concurrent reassembly slots: the network has 3 nodes, so at most 3
 * peers can be transmitting at once; a 4th slot was pure RAM. */
#define MAX_CONCURRENT_BURSTS 3
#define BURST_REASM_PAYLOAD_MAX 12288  /* max payload per burst (consensus + bytecode txns) */
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
    uint32_t burst_id_conflicts; /* RX packets whose burst_id matched a slot held by a DIFFERENT burst (per-node counter + boot-random XOR offsets collide) */
} g_stats = { 0 };

/* ----------------------------------------------------------------------------
 * Deferred host UART transfer queue + dedicated TX consumer thread.
 *
 * The radio callback (rac_post_callback) runs inside smtc_rac_run_engine(),
 * which is polled in the main superloop. Doing the byte-polled UART transfer
 * (usb_tx) synchronously — in the callback OR in the superloop — blocks the
 * radio engine for ~134ms per 12.4KB frame (measured: event-23 loop
 * iterations of 143ms, event-24 hw-vs-app RX divergence of 23 packets: the
 * radio HEARD peers' frames the engine never processed). With phase-locked
 * multi-node traffic that starvation is fatal: a node transmitting in
 * phase 3 goes blind to peers' proposals and phase-4 deliveries for the
 * remainder of the window (observed 3.2s post-TX receive blackout).
 *
 * Correct pattern (symmetric with the RX ISR ring): producers (radio
 * callback, command handler — both in the main thread) copy frames into
 * this ring and post a semaphore; a DEDICATED lower-priority preemptible
 * thread is the sole consumer, streaming slots to the UART while the main
 * thread keeps polling the radio engine every ~1ms. The main loop no
 * longer performs any UART TX work.
 *
 * Ownership: producers own `tail` (and may only append; on a full ring the
 * NEW frame is dropped — never steal the consumer's head slot, which would
 * race the consumer's in-flight copy). Consumer owns `head` exclusively.
 * The semaphore provides the wakeup + ordering barrier.
 * ------------------------------------------------------------------------- */
/* Hostbound (MCU→host) frame ring: 3 slots, each sized to carry a FULL
 * reassembled 12,288B RX frame (3B 'R' header + payload + 2B RSSI +
 * margin). The old 2048B slots silently dropped every large burst before
 * it reached the host — capping the whole network at ~2KB frames. */
#define HOST_TX_QUEUE_DEPTH 3
#define HOST_TX_SLOT_BYTES  12400
static uint8_t  g_host_tx_buf[HOST_TX_QUEUE_DEPTH][HOST_TX_SLOT_BYTES];
static uint16_t g_host_tx_len[HOST_TX_QUEUE_DEPTH];
static volatile uint8_t g_host_tx_head;   /* TX thread consumes (advances on drain) */
static volatile uint8_t g_host_tx_tail;   /* main thread produces (advances on queue) */
/* Scratch buffer for atomic frame drain: copied from the ring slot before
 * usb_tx streams it, so queue_host_tx can't overwrite mid-transfer. Owned
 * exclusively by the TX consumer thread. */
static uint8_t  g_host_tx_scratch[HOST_TX_SLOT_BYTES];

/* TX consumer thread: sole drainer of the host-TX ring. */
static struct k_sem g_host_tx_sem;
static void usb_tx_thread( void* p1, void* p2, void* p3 );
static K_THREAD_STACK_DEFINE( usb_tx_thread_stack, 2048 );
static struct k_thread usb_tx_thread_cb;

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

/* ----------------------------------------------------------------------- *
 * Interrupt-driven UART RX ring buffer (hw_modem pattern).
 *
 * Poll-mode RX on the nRF UARTE has ~6 bytes of buffering (5-entry HW FIFO
 * + 1-byte poll DMA ≈ 65µs of line time at 921600 baud). Any main-loop
 * window longer than that — radio IRQ processing 0.5-1.5ms, a host-TX
 * drain 5-24ms — silently dropped bytes during host writes, capping
 * reliable frames at ~1KB. The UART ISR now drains the FIFO into this
 * ring regardless of main-loop state; usb_rx_poll() consumes it. The
 * hw_modem reference sample (samples/usp/rac/hw_modem) uses this exact
 * uart_irq_* contract on the same driver stack.
 *
 * Single producer (ISR) / single consumer (main loop): volatile head/tail
 * with power-of-2 masking needs no lock. usb_rx_poll() must be the ONLY
 * consumer, and the ISR the ONLY place calling uart_fifo_read — mixing
 * uart_poll_in with the IRQ path would contend for the driver's 1-byte
 * DMA buffer.
 * ----------------------------------------------------------------------- */
#define UART_RX_RING_SIZE 8192  /* power of 2 */
static uint8_t  g_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t g_rx_ring_head = 0; /* ISR writes */
static volatile uint16_t g_rx_ring_tail = 0; /* main loop reads */
static volatile uint32_t g_rx_ring_dropped = 0;

static void uart_rx_isr( const struct device* dev, void* user_data )
{
    ARG_UNUSED( user_data );

    if( !uart_irq_update( dev ) )
    {
        return;
    }

    while( uart_irq_rx_ready( dev ) )
    {
        uint8_t b;
        int n = uart_fifo_read( dev, &b, 1 );
        if( n <= 0 )
        {
            break;
        }
        uint16_t next = ( g_rx_ring_head + 1 ) & ( UART_RX_RING_SIZE - 1 );
        if( next == g_rx_ring_tail )
        {
            /* Ring full — drop rather than overwrite in-flight data. */
            g_rx_ring_dropped++;
            break;
        }
        g_rx_ring[g_rx_ring_head] = b;
        g_rx_ring_head = next;
    }
}

static int usb_init( void )
{
    if( !device_is_ready( cdc_dev ) ) return -ENODEV;

    uart_irq_callback_user_data_set( cdc_dev, uart_rx_isr, NULL );
    uart_irq_rx_enable( cdc_dev );

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
 * Stage a frame for deferred UART transfer to the host. Up to three segments
 * (header / payload / trailer) are concatenated into one ring slot. Called
 * from rac_post_callback and usb_rx_poll (both main-thread context) — never
 * blocks. If the frame won't fit a slot or the ring is full, it is dropped
 * and counted in g_stats.host_tx_dropped. On a FULL ring the NEW frame is
 * dropped (not the oldest): the consumer thread owns `head` exclusively,
 * and stealing its in-flight slot from producer context would tear the
 * frame being streamed.
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
        /* Ring full — drop this frame; the TX thread will catch up. */
        g_stats.host_tx_dropped++;
        return;
    }

    uint8_t*  dst = g_host_tx_buf[g_host_tx_tail];
    uint16_t  off = 0;
    if( len1 ) { memcpy( dst + off, seg1, len1 ); off += len1; }
    if( len2 ) { memcpy( dst + off, seg2, len2 ); off += len2; }
    if( len3 ) { memcpy( dst + off, seg3, len3 ); off += len3; }
    g_host_tx_len[g_host_tx_tail] = off;
    g_host_tx_tail = next;
    k_sem_give( &g_host_tx_sem );
}

/*
 * Dedicated UART TX consumer. Runs at the first preemptible priority, BELOW
 * the (cooperative) main thread: it streams queued frames to the host only
 * while the main thread sleeps between engine polls, so the radio engine is
 * polled every ~1ms regardless of UART transfer length. This replaces the
 * old in-superloop drain, whose ~134ms busy-poll per 12.4KB frame starved
 * the engine (event-23 143ms iterations; event-24 hw-heard-but-unprocessed
 * packets — the root cause of the 3.2s post-TX receive blackout).
 */
static void usb_tx_thread( void* p1, void* p2, void* p3 )
{
    ARG_UNUSED( p1 ); ARG_UNUSED( p2 ); ARG_UNUSED( p3 );

    while( true )
    {
        k_sem_take( &g_host_tx_sem, K_FOREVER );
        while( g_host_tx_head != g_host_tx_tail )
        {
            uint16_t tx_len = g_host_tx_len[g_host_tx_head];
            /* Copy out of the ring slot BEFORE releasing it: the producer
             * may reuse the slot the instant head advances. */
            memcpy( g_host_tx_scratch, g_host_tx_buf[g_host_tx_head], tx_len );
            g_host_tx_head = (uint8_t)( ( g_host_tx_head + 1 ) % HOST_TX_QUEUE_DEPTH );
            ( void ) usb_tx( g_host_tx_scratch, tx_len );
        }
    }
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
    /* Extended stats: 48 bytes (16 counters + 12 diagnostic + MCU time +
     * ring drops + staging overwrites). Bytes 44-47 reserved (zero). */
    uint8_t header[3] = { FLRC_MSG_STATS, 48, 0 };
    uint8_t stats_bytes[48] = { 0 };
    sys_put_le32( g_stats.bursts_tx, &stats_bytes[0] );
    sys_put_le32( g_stats.bursts_rx, &stats_bytes[4] );
    sys_put_le32( g_stats.packets_tx, &stats_bytes[8] );
    sys_put_le32( g_stats.packets_rx, &stats_bytes[12] );
    sys_put_le32( g_stats.crc_errors, &stats_bytes[16] );
    sys_put_le32( g_stats.host_tx_dropped, &stats_bytes[20] );
    /* Diagnostic fields for TX degradation investigation */
    stats_bytes[24] = (uint8_t) g_burst_mode;           /* 0=RX, 1=ABORTING, 2=TX */
    stats_bytes[25] = g_tx_repeat_count;                 /* LBT backoff counter */
    stats_bytes[26] = g_pending_tx_valid ? 1 : 0;        /* TX payload pending? */
    stats_bytes[27] = g_restart_rx_pending ? 1 : 0;      /* RX restart needed? */
    sys_put_le32( smtc_modem_hal_get_time_in_ms( ) - g_tx_backoff_until_ms, &stats_bytes[28] ); /* ms until backoff clears (negative if past) */
    sys_put_le32( smtc_modem_hal_get_time_in_ms( ), &stats_bytes[32] );  /* current MCU time */
    sys_put_le32( g_rx_ring_dropped, &stats_bytes[36] ); /* UART RX ring drops (must stay 0) */
    sys_put_le32( g_staging_overwrites, &stats_bytes[40] ); /* TX queue overruns (must stay 0) */
    sys_put_le32( g_stats.burst_id_conflicts, &stats_bytes[44] ); /* burst-id collisions (see slot matching) */
    queue_host_tx( header, 3, stats_bytes, sizeof( stats_bytes ), NULL, 0 );
}

static void send_time_packet( void )
{
    uint8_t pkt[5];
    pkt[0] = FLRC_MSG_TIME;
    sys_put_le32( smtc_modem_hal_get_time_in_ms( ), &pkt[1] );
    queue_host_tx( pkt, sizeof( pkt ), NULL, 0, NULL, 0 );
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

    if( g_last_tx_complete_ms != 0 )
    {
        send_debug_event( 22, smtc_modem_hal_get_time_in_ms( ) - g_last_tx_complete_ms );
        g_last_tx_complete_ms = 0;
    }

    g_rac_ctx->scheduler_config.scheduling    = SMTC_RAC_ASAP_TRANSACTION;
    /* Use current time — same fix as TX: prevents the 128s failsafe from
     * triggering based on boot time for long-running RX transactions. */
    g_rac_ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
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
    /* Use current time as start_time_ms. The radio planner's failsafe checks
     * start_time_ms + 128000 < now to detect stuck tasks. With start_time_ms=0
     * (boot time), ANY running task triggers the failsafe after 128s of uptime.
     * Using current time ensures the failsafe window is relative to each TX. */
    g_rac_ctx->scheduler_config.start_time_ms = smtc_modem_hal_get_time_in_ms( );
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
    g_tx_start_time_ms = smtc_modem_hal_get_time_in_ms( );
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
                        /* TUPLE KEY (2026-08-22 bench forensics): the slot
                         * must belong to THIS burst, not merely share its
                         * 16-bit id. burst_ids are per-node counters XOR'd
                         * with a boot-random offset — two nodes collide
                         * regularly, and id-only matching fed the second
                         * sender's packets into the first sender's
                         * incomplete slot: received_count never reached
                         * total_packets, the burst rotted until timeout,
                         * while packets_rx counted every packet (event-24
                         * hw-rx-not-app-delivered matched the lost-burst
                         * packet counts exactly; delivery was bimodal per
                         * pair — 99% vs 16-20% — re-randomized every
                         * reboot). The payload CRC32 makes a cross-sender
                         * tuple match implausible. */
                        if( g_rx_slots[i].total_packets == total_packets &&
                            g_rx_slots[i].payload_len   == total_payload_len &&
                            g_rx_slots[i].payload_crc   == payload_crc32 )
                        {
                            slot = &g_rx_slots[i];
                            break;
                        }
                        g_stats.burst_id_conflicts++;
                        send_debug_event( 26, burst_id );
                        /* Different sender's burst under the same id — it
                         * takes its own slot below. */
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
        send_debug_event( 25, g_tx_repeat_count );
        if( g_tx_repeat_count > 10 )
        {
            /* Channel persistently busy — drop this frame to avoid
             * starving the radio with endless retries. RESTART RX: the
             * RX transaction was aborted to seize the radio for this TX,
             * and with the frame now dropped nothing else will ever set
             * g_restart_rx_pending — the radio stayed DEAF until some
             * later TX happened to succeed (observed: 30s of near-zero
             * btx/brx under phase-aligned peer traffic). Safe here: no
             * TX is pending, so the superloop cannot re-abort in a loop. */
            g_pending_tx_valid = false;
            g_pending_tx_len   = 0;
            g_tx_repeat_count  = 0;
            g_restart_rx_pending = true;
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
        else if( g_burst_mode == BURST_TX_ABORTING )
        {
            /* Abort completed but the frame is gone (dropped/watchdog):
             * without this fallback the mode stays BURST_TX_ABORTING
             * forever — the TX trigger and the RX restart both require
             * BURST_RX, wedging TX and RX permanently. */
            g_burst_mode         = BURST_RX;
            g_restart_rx_pending = true;
        }
        else if( g_burst_mode == BURST_TX )
        {
            LOG_INF( "TX burst complete" );
            send_debug_event( 8, 0 ); /* Debug event 8: TX complete */
            g_last_tx_complete_ms = smtc_modem_hal_get_time_in_ms( );
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
    while( g_rx_ring_tail != g_rx_ring_head )
    {
        b = g_rx_ring[g_rx_ring_tail];
        g_rx_ring_tail = ( g_rx_ring_tail + 1 ) & ( UART_RX_RING_SIZE - 1 );
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
            else if( tag == FLRC_MSG_RESET )
            {
                /* Host requests a system reboot. Send an ACK directly
                 * (not via the deferred queue, which would be lost on
                 * reboot), then cold-reset the MCU. This recovers from
                 * a stuck RAC / corrupted radio state. */
                LOG_INF( "Reset requested by host — rebooting" );
                uint8_t ack = FLRC_MSG_READY;
                usb_tx( &ack, 1 );
                k_busy_wait( 10000 ); /* 10ms for USB flush */
                NVIC_SystemReset( );
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
            g_staging_tx_buf[have++] = b;
            if( have >= frame_len )
            {
                if( g_staging_tx_valid )
                {
                    g_staging_overwrites++; /* host overran the 2-deep queue */
                }
                g_staging_tx_len = frame_len;
                g_pending_burst_id++;
                /* Apply per-node random offset so different nodes don't
                 * collide on the same burst_id. Without this, two nodes
                 * incrementing from 0 produce overlapping burst_ids, and
                 * the receiver's multi-slot reassembly mixes their packets
                 * into the same slot → CRC failure on both. */
                /* (actual offset applied at TX header build time) */
                g_staging_tx_valid = true;
                send_debug_event( 20, g_staging_tx_len ); /* Debug event 20: Payload enqueued */
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

    /* Start the dedicated UART TX consumer thread BEFORE anything queues a
     * hostbound frame (send_ready below): first preemptible priority, so it
     * only runs while the main (cooperative) thread sleeps — the radio
     * engine's poll cadence is never hostage to UART transfer length. */
    k_sem_init( &g_host_tx_sem, 0, HOST_TX_QUEUE_DEPTH );
    k_thread_create( &usb_tx_thread_cb, usb_tx_thread_stack,
                     K_THREAD_STACK_SIZEOF( usb_tx_thread_stack ),
                     usb_tx_thread, NULL, NULL, NULL,
                     K_PRIO_PREEMPT( 0 ), 0, K_NO_WAIT );
    k_thread_name_set( &usb_tx_thread_cb, "usb_tx" );

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
        /* Event 23: measure the PREVIOUS iteration's duration. A long
         * iteration means smtc_rac_run_engine was not polled for that
         * long — the engine-starvation signature (usb_tx drains, etc.). */
        {
            uint32_t now = smtc_modem_hal_get_time_in_ms( );
            if( g_loop_iter_start_ms != 0 && now - g_loop_iter_start_ms > 20 )
            {
                send_debug_event( 23, now - g_loop_iter_start_ms );
            }
            g_loop_iter_start_ms = now;
        }
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

            /* TX queue promotion: move a completed staging frame into the
             * pending (air) buffer when the radio isn't holding one. This
             * is what lets the host pipeline: it may stream frame N+1 over
             * UART while frame N is still airing. */
            if( !g_pending_tx_valid && g_staging_tx_valid )
            {
                memcpy( g_pending_tx_buf, g_staging_tx_buf, g_staging_tx_len );
                g_pending_tx_len   = g_staging_tx_len;
                g_pending_tx_valid = true;
                g_staging_tx_valid = false;
                g_staging_tx_len   = 0;
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

            /* TX watchdog: if a TX burst hasn't completed within the timeout,
             * the RAC engine's TX transaction has hung (observed after ~115s
             * of sustained operation). Abort the stuck transaction and return
             * to RX mode so the radio can recover. Without this, g_burst_mode
             * stays BURST_TX forever and all TX/RX stops. */
            if( g_burst_mode == BURST_TX )
            {
                uint32_t tx_elapsed = smtc_modem_hal_get_time_in_ms( ) - g_tx_start_time_ms;
                if( tx_elapsed > TX_WATCHDOG_TIMEOUT_MS )
                {
                    LOG_INF( "TX watchdog: aborting stuck TX burst (elapsed=%ums)", tx_elapsed );
                    g_pending_tx_valid = false;
                    g_pending_tx_len   = 0;
                    g_tx_repeat_count  = 0;
                    g_burst_mode       = BURST_RX;
                    g_restart_rx_pending = true;
                    ( void ) smtc_rac_abort_radio_submit( g_radio_access_id );
                }
            }

            /* Emit periodic stats every 10s so the host can track counter
             * evolution (especially host_tx_dropped) without polling. */
            {
                uint32_t now_ms = smtc_modem_hal_get_time_in_ms( );
                if( now_ms - g_last_stats_emit_ms >= STATS_EMIT_INTERVAL_MS )
                {
                    g_last_stats_emit_ms = now_ms;
                    send_stats_packet( );
                    /* Event 24: hardware-received minus app-delivered packets
                     * over this interval. Positive divergence = the radio
                     * heard packets the engine never processed (starvation). */
                    lr20xx_radio_flrc_rx_stats_t hw;
                    lr20xx_radio_flrc_get_rx_stats(
                        smtc_rac_get_radio_driver_context( ), &hw );
                    uint32_t hw_delta = hw.received_packets - g_hw_rx_last;
                    uint32_t app_delta = g_stats.packets_rx - g_app_rx_last;
                    g_hw_rx_last = hw.received_packets;
                    g_app_rx_last = g_stats.packets_rx;
                    if( hw_delta > app_delta )
                    {
                        send_debug_event( 24, hw_delta - app_delta );
                    }
                }
            }

            /* Host-TX drain moved to the dedicated usb_tx_thread (see its
             * comment): the superloop no longer performs ANY UART TX work.
             * The old in-loop drain busy-polled ~134ms per 12.4KB frame,
             * starving the radio engine (event-23 143ms iterations,
             * event-24 hw-heard-but-unprocessed packets) — the measured
             * cause of the post-TX receive blackout. */
        }

        /* Sleep, not k_yield: the main thread is cooperative, and the TX
         * consumer runs at the first preemptible priority — it only gets
         * the CPU while we sleep. 1ms bounds both the engine-poll gap and
         * the RX-ring service latency (~125 bytes at 1M baud, trivially
         * inside the 8KB ring). */
        k_msleep( 1 );
    }

    return 0;
}
