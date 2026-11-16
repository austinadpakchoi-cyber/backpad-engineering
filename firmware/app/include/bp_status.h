#ifndef BP_STATUS_H
#define BP_STATUS_H
#include <stdint.h>
#include <stdbool.h>
/* Read with a brief halted snapshot, or require equal even snapshot_seq before/after.
 * All values are local observations, never proof of host delivery. */
typedef struct {
 uint32_t magic, version, snapshot_seq, uptime_ms, phase;
 uint32_t device_id[2], reset_reason;
 int32_t last_error;
 uint32_t epoch, connected, encrypted, subscribed, report_mode, suspended;
 uint32_t sensor_frames, sensor_no_data, sensor_faults, sensor_session, sensor_sequence;
 uint32_t sensor_count; int32_t sensor_x, sensor_y;
 uint32_t pointer_mode, tx_accepted, tx_completed, tx_errors, cancellations;
 uint32_t sensor_init_calls;
 uint32_t hfclkstat, lfclkstat, dcdcen, dcdcen0;
 int32_t bt_disable_error;
} bp_test_status;
extern volatile bp_test_status backpad_status;
/* Single main-thread publisher. Only bounded memory copy is inside the seqlock;
 * no waits, peripheral access, or sensor/Bluetooth operations occur while odd. */
#define BP_STATUS_PERIOD_MS 100u
bool bp_status_publish_due(volatile bp_test_status *published,
                           const bp_test_status *working, uint64_t now,
                           uint64_t *next_due, bool force);
/* phase: 1 boot, 2 sensor ready, 3 advertising, 4 connected, 5 HID ready, 255 halted */
#endif

