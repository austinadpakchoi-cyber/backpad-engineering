#ifndef BP_IQS9151_H
#define BP_IQS9151_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
/* Azoteq IQS9151 (datasheet v1.2, Oct 2026). I2C 7-bit 0x56, 16-bit register address sent LOW
 * byte first, little-endian data. RDY is open-drain ACTIVE LOW (4.7k pull-up on the BackPad V2
 * touchpad). With Config Settings bit4 (Force Comms Method)=1, as in the evaluation-kit settings,
 * any transaction while RDY is high is ACKed but ignored: reads return 0xEE. Every transaction
 * therefore happens inside a RDY-low window; an I2C STOP closes the window. */
#define BP_IQS9151_ADDR            0x56u
#define BP_IQS9151_PRODUCT         0x09BCu
#define BP_IQS9151_REG_PRODUCT     0x1000u  /* product, major, minor: 3 x u16 */
#define BP_IQS9151_REG_BLOCK       0x1014u  /* rel X .. finger 7 area, read every window */
#define BP_IQS9151_BLOCK_BYTES     72u      /* 0x1014..0x105B */
#define BP_IQS9151_REG_INFO        0x1020u
#define BP_IQS9151_REG_SYSCTRL     0x11BCu
#define BP_IQS9151_REG_CONFIG      0x11BEu
#define BP_IQS9151_REG_TP_SETTINGS 0x11E2u  /* settings byte, total Rx, total Tx, max touches, X res, Y res */
#define BP_IQS9151_REG_MAPPING     0x1218u
#define BP_IQS9151_REG_TOTALS      0x11E3u  /* total Rx, total Tx */
#define BP_IQS9151_REG_MAX_TOUCHES 0x11E5u
#define BP_IQS9151_REG_RESOLUTION  0x11E6u  /* X res, Y res (LE16 each) */
#define BP_IQS9151_MAP_ENTRIES     25u      /* IQS9151: at most 25 electrodes */
#define BP_IQS9151_MAX_FINGERS     7u

/* Info Flags (0x1020) */
#define BP_IQS9151_INFO_ATI_ERROR   (1u<<3)
#define BP_IQS9151_INFO_REATI_DONE  (1u<<4)
#define BP_IQS9151_INFO_SHOW_RESET  (1u<<7)
#define BP_IQS9151_INFO_GLOBAL_TOUCH (1u<<9)
/* Trackpad Flags (0x1022) */
#define BP_IQS9151_TP_FINGERS_MASK  0x0Fu
#define BP_IQS9151_TP_MOVEMENT      (1u<<4)
#define BP_IQS9151_TP_TOO_MANY      (1u<<5)
#define BP_IQS9151_TP_SATURATION    (1u<<7)
#define BP_IQS9151_TP_CONFIDENCE(k) (1u<<(8u+(k)))
/* System Control (0x11BC) */
#define BP_IQS9151_SC_TP_RESEED     (1u<<3)
#define BP_IQS9151_SC_TP_REATI      (1u<<5)
#define BP_IQS9151_SC_ACK_RESET     (1u<<7)
#define BP_IQS9151_SC_SW_RESET      (1u<<9)
/* Trackpad Settings byte (0x11E2) */
#define BP_IQS9151_TPS_FLIP_X       (1u<<0)
#define BP_IQS9151_TPS_FLIP_Y       (1u<<1)
#define BP_IQS9151_TPS_SWITCH_XY    (1u<<2)
#define BP_IQS9151_TPS_IIR          (1u<<3)
#define BP_IQS9151_TPS_IIR_STATIC   (1u<<4)
#define BP_IQS9151_TPS_JITTER       (1u<<5)
#define BP_IQS9151_TPS_AREA_OFF     (1u<<6)

typedef struct { uint16_t reg; const uint8_t *data; uint16_t len; } bp_iqs9151_section;
extern const bp_iqs9151_section bp_iqs9151_settings[];
extern const unsigned bp_iqs9151_settings_count;

typedef struct { uint16_t x, y, strength, area; bool valid; } bp_iqs9151_finger;
typedef struct {
    int16_t rel_x, rel_y;
    uint16_t gesture_x, gesture_y, gestures1, gestures2, info, tp;
    uint8_t fingers_reported;  /* Trackpad Flags low nibble */
    bp_iqs9151_finger finger[BP_IQS9151_MAX_FINGERS];  /* slot k is the chip's tracked finger k+1 */
} bp_iqs9151_frame;

typedef enum { BP_IQS9151_OK=0, BP_IQS9151_ARGUMENT, BP_IQS9151_LENGTH, BP_IQS9151_COMMS, BP_IQS9151_COUNT,
               BP_IQS9151_COORDINATE } bp_iqs9151_result;
/* Decodes the 72-byte block read from 0x1014. An all-0xEE block (read outside a RDY window)
 * is BP_IQS9151_COMMS, never data. Slot coordinates 0xFFFF mean "no finger in this slot".
 * Coordinates beyond the configured resolution are BP_IQS9151_COORDINATE. A slot with zero strength
 * and zero area is empty too: slots beyond Max Multi-Touches read 0x0000, not 0xFFFF. */
bp_iqs9151_result bp_iqs9151_decode(const uint8_t *raw, size_t n, uint16_t x_res, uint16_t y_res,
                                    bp_iqs9151_frame *out);

/* Host abstraction. read/write perform exactly ONE transaction inside the current fresh RDY-low
 * window and consume it (STOP closes the window). -EAGAIN: no fresh window yet. */
typedef struct { uint32_t epoch; uint64_t ready_ms, done_ms; } bp_iqs9151_window;
typedef struct {
    void *context;
    int (*read)(void *, uint16_t reg, uint8_t *data, size_t n, bp_iqs9151_window *w);
    int (*write)(void *, uint16_t reg, const uint8_t *data, size_t n, bp_iqs9151_window *w);
    uint64_t (*now_ms)(void *);
    void (*wait_ms)(void *, unsigned ms);
} bp_iqs9151_hal;

#define BP_IQS9151_WINDOW_TIMEOUT_MS 200u   /* streaming windows come every ~5 ms */
#define BP_IQS9151_RESET_WAIT_MS     100u
#define BP_IQS9151_ATI_TIMEOUT_MS    3000u
#define BP_IQS9151_MAX_SECTION       256u
#define BP_IQS9151_MAX_READ          320u   /* one channel table: 13 x 12 x 2 = 312 bytes */
#define BP_IQS9151_REG_COUNTS        0xA000u
#define BP_IQS9151_REG_REFERENCES    0xB000u
#define BP_IQS9151_REG_DELTAS        0xC000u
#define BP_IQS9151_REG_ATI_COMP      0xD000u
#define BP_IQS9151_REG_CH_DISABLE    0x1246u  /* one LE32 per Tx row (mapping order), bit = Rx column */
#define BP_IQS9151_MAX_TX            22u
#define BP_IQS9151_COUNT_FULL_SCALE  2047u
/* More dead channels than this is not a few bad pins (e.g. a finger on a bare pad at start-up):
 * nothing is disabled and the ATI error stays visible. Board 1 has 36 (two Tx rows + one Rx column). */
#define BP_IQS9151_MAX_DEAD          48u

/* Init progress, published for diagnostics (which step failed). */
typedef enum { BP_IQS9151_STEP_NONE=0, BP_IQS9151_STEP_PRODUCT, BP_IQS9151_STEP_RESET, BP_IQS9151_STEP_ACK,
               BP_IQS9151_STEP_SETTINGS, BP_IQS9151_STEP_VERIFY, BP_IQS9151_STEP_ATI, BP_IQS9151_STEP_ATI_WAIT,
               BP_IQS9151_STEP_RESEED, BP_IQS9151_STEP_STREAM } bp_iqs9151_step;

typedef struct {
    bp_iqs9151_hal hal;
    bool initialized;
    uint32_t session, next_session, sequence;
    uint16_t product, major, minor, x_res, y_res;
    uint8_t max_touches;          /* 0x11E5 as read back: slots beyond it are not maintained by the chip */
    uint8_t total_rx, total_tx;   /* Total Rx/Tx in use (after dead lines are taken out) */
    uint8_t orig_rx, orig_tx;     /* Total Rx/Tx of the settings table (13 x 12) */
    uint8_t col_phys[32], row_phys[BP_IQS9151_MAX_TX]; /* settings-table index of each Rx column / Tx row in use */
    uint8_t removed_rx, removed_tx; /* Rx columns / Tx rows dead across the whole pad, taken out of the mapping */
    uint8_t dead_channels;        /* channels at full scale after the first ATI */
    uint8_t dead_handled;         /* bit0: isolated dead channels disabled, bit1: dead lines removed; ATI repeated */
    uint32_t dead_rows[BP_IQS9151_MAX_TX]; /* per settings-table Tx row: bit = settings-table Rx column found dead */
    uint16_t last_info, last_tp;
    uint8_t tp_settings;          /* current 0x11E2 byte (flip/switch/filter bits) */
    uint8_t step, sw_resets;
    uint32_t ati_ms;              /* duration of the last trackpad ATI */
    bool ati_error;               /* chip reported ATI Error after the last ATI (non-fatal, published) */
    int32_t last_error;
    uint8_t last_block[BP_IQS9151_BLOCK_BYTES];
    uint64_t last_block_ms;
} bp_iqs9151_bus;

/* Full start-up: product check, (soft reset if no Show Reset), Ack Reset, write every settings
 * section, read back the trackpad geometry and mapping, TP re-ATI with ATI-error check, reseed.
 * After an ATI error, Tx rows / Rx columns at full scale across the whole pad are taken out of the
 * mapping, other channels at full scale are disabled, and ATI runs once more. */
int bp_iqs9151_bus_init(bp_iqs9151_bus *, const bp_iqs9151_hal *);
/* One streaming window: reads the block and decodes it. -EAGAIN no window; -ECONNRESET the chip
 * reports Show Reset (caller must re-initialize); -EBADMSG undecodable block. */
int bp_iqs9151_bus_read(bp_iqs9151_bus *, bp_iqs9151_frame *out, bp_iqs9151_window *w);
/* Runtime register write with readback verification (each in its own window). The caller is
 * responsible for the whitelist. */
int bp_iqs9151_bus_write_verify(bp_iqs9151_bus *, uint16_t reg, const uint8_t *data, size_t n);
int bp_iqs9151_bus_read_reg(bp_iqs9151_bus *, uint16_t reg, uint8_t *data, size_t n);
/* The chip reported ATI Error after an ATI (a measurement result, not a bus failure: bus errors keep
 * their own codes such as -EIO from I2C). */
#define BP_IQS9151_EATI (-ERANGE)
/* Queue a trackpad re-ATI and wait for completion (windows may pause during ATI: waits up to
 * BP_IQS9151_ATI_TIMEOUT_MS); BP_IQS9151_EATI on ATI error. */
int bp_iqs9151_bus_reati(bp_iqs9151_bus *);
void bp_iqs9151_bus_cancel(bp_iqs9151_bus *);
#endif

