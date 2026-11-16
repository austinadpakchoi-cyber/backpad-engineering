#ifndef BP_TUNE_H
#define BP_TUNE_H
#include <stdint.h>
#include "bp_touch_policy.h"
#include "bp_trace_codec.h"
#define BP_TUNE_MAX_FRAMES 1600u
/* Host (J-Link) mailbox. Host writes op,arg,expected (and backpad_iqs_write for op9) first, then
 * a NEW nonzero request last. Firmware acknowledges only after completion; a repeated request is
 * ignored. A nonzero result is never success.
 *   op1  capture arg (1..BP_TUNE_MAX_FRAMES) frames into backpad_tune_capture
 *   op2  wait for a zero-contact frame, then capture arg frames starting at the next raw contact;
 *        arming timeout30s, recording timeout10s. State1/count0 means armed, not complete.
 *   op3  full chip re-initialization (settings + ATI)
 *   op4  read IQS9151 memory into backpad_iqs_read (counts/deltas/references/ATI compensation)
 *   op5  trackpad re-ATI; refused (-EBUSY) unless no finger has been reported for 300 ms
 *   op6  lossless full-rate timed raw trace, arg=duration_ms(1000..40000).
 *        Version3; reserved[0]=payload bytes,reserved[1]=duration,reserved[2]=session.
 *   op9  write backpad_iqs_write.len (1..16) bytes at backpad_iqs_write.reg with readback;
 *        only whitelisted tuning registers (thresholds, filters, orientation, ATI target, rate)
 * Touch-admission parameters need no op: write backpad_iqs_params fields, then increment its seq. */
typedef struct {
 uint32_t request, op, arg, expected;
 uint32_t ack; int32_t result;
 uint32_t state, reserved;
} bp_tune_control;
/* IQS9151 capture record (version 2). xy[36] packs the 0x1014 block:
 *   xy[0..3]   = block[8..11]   single/two-finger gesture flags (0x101C..0x101F)
 *   xy[4..31]  = block[12..39]  info flags, trackpad flags, fingers 1..3 (x,y,strength,area LE16)
 *   xy[32..35] = block[0..3]    relative X/Y (0x1014..0x1017) */
typedef struct { uint32_t ms, sequence, session; uint8_t xy[36]; } bp_tune_record;
typedef struct {
 uint32_t magic, version, snapshot_seq, request, state, count, target;
 uint32_t start_ms, end_ms; int32_t error;
 uint32_t multiplier, shift, reserved[4];
 /* config: product,major,minor (LE16 x3), tp_settings, x_res, y_res (LE16), ati_error, ati_ms (LE32),
  * then the 25-byte RxTx mapping as read back, then dead-channel count, handling bits (bit0 disabled,
  * bit1 lines removed), removed Tx rows, removed Rx columns (bytes 41..44). */
 uint8_t config[64];
 union {
  bp_tune_record records[BP_TUNE_MAX_FRAMES];
  uint8_t compressed[BP_TRACE_BYTES];
 };
} bp_tune_capture;
#define BP_TUNE_CAPTURE_VERSION_IQS9151 2u
typedef struct { uint16_t reg; uint8_t len, reserved; uint8_t data[16]; } bp_iqs_write;
/* op4: read len (1..320) bytes of IQS9151 memory at reg (any address; reads are side-effect free),
 * e.g. 0xA000 counts, 0xB000 references, 0xC000 deltas, 0xD000 ATI compensation (156 x LE16). */
typedef struct { uint16_t reg, len; int32_t result; uint32_t done_ms; uint8_t data[320]; } bp_iqs_read;
/* Published every frame under a sequence lock (odd = being written). */
typedef struct {
 uint32_t seq;
 uint8_t step, ati_error, tp_settings, fingers;
 int32_t last_error;
 uint16_t product, major, minor, x_res, y_res, info, tp;
 uint8_t dead_channels, dead_handled; /* channels at full scale after start-up ATI; bit0 disabled, bit1 lines removed */
 uint32_t session, sequence, frames, missed_windows, bad_frames, chip_resets, init_calls, force_requests;
 uint32_t ati_ms, admitted_frames, suppressed_frames, uptime_ms;
 struct { uint16_t x, y, strength, area; } finger[3];
 uint8_t block[72];
} bp_iqs_diag;
_Static_assert(sizeof(bp_tune_record)==48,"record ABI");
_Static_assert(sizeof(bp_tune_capture)==128u+BP_TRACE_BYTES,"capture ABI");
_Static_assert(sizeof(bp_tune_control)==32,"control ABI");
_Static_assert(sizeof(bp_iqs_write)==20,"write ABI");
_Static_assert(sizeof(bp_iqs_read)==332,"read ABI");
_Static_assert(sizeof(bp_iqs_diag)==172,"diag ABI");
_Static_assert(sizeof(bp_touch_params)==20,"params ABI");
_Static_assert(sizeof(bp_touch_filter_params)==24,"filter params ABI");
#endif

