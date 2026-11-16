#ifndef BP_HID_H
#define BP_HID_H
#include "bp_pointer.h"
#define BP_HID_REPORT_ID 1u
#define BP_HID_PAYLOAD_SIZE 4u
extern const uint8_t bp_hid_report_map[];
extern const size_t bp_hid_report_map_size;
bp_result bp_hid_encode(const bp_report *,uint8_t out[BP_HID_PAYLOAD_SIZE]);
/* Framing contract only: no USB device implementation is supplied. */
bp_result bp_hid_encode_with_id(const bp_report *,uint8_t out[5]);
#endif

