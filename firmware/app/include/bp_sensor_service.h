#ifndef BP_SENSOR_SERVICE_H
#define BP_SENSOR_SERVICE_H
#include "bp_pointer.h"
/* Sole sensor worker owns bus, ATI, diagnostics and raw captures. BLE/main
 * consume queued processed samples only; no sensor calls from callbacks. */
typedef struct {
 uint32_t frames,no_data,faults,init_calls,recoveries,calibrations;
 uint32_t state,raw_count,filtered_count,off_frames;
 int32_t last_error,bus_error;
} bp_sensor_health;
void bp_sensor_start(void);
bool bp_sensor_take(bp_sample *);
void bp_sensor_health_get(bp_sensor_health *);
#endif

