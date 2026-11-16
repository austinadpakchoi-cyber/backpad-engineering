#include "bp_hid.h"
/* Generic Desktop / Mouse; Pointer; 3 buttons, 5 padding bits; relative
 * signed 8-bit X/Y/Wheel. HID short items, report protocol only. */
const uint8_t bp_hid_report_map[]={
    0x05,0x01,0x09,0x02,0xa1,0x01,0x85,BP_HID_REPORT_ID,
    0x09,0x01,0xa1,0x00,0x05,0x09,0x19,0x01,0x29,0x03,
    0x15,0x00,0x25,0x01,0x95,0x03,0x75,0x01,0x81,0x02,
    0x95,0x01,0x75,0x05,0x81,0x03,
    0x05,0x01,0x09,0x30,0x09,0x31,0x09,0x38,
    0x15,0x81,0x25,0x7f,0x75,0x08,0x95,0x03,0x81,0x06,0xc0,0xc0
};
const size_t bp_hid_report_map_size=sizeof(bp_hid_report_map);
bp_result bp_hid_encode(const bp_report *r,uint8_t out[4]) {
    if(r==NULL || out==NULL || r->buttons>7u || r->dx < -127 || r->dx>127 ||
       r->dy < -127 || r->dy>127 || r->wheel < -127 || r->wheel>127) return BP_INVALID;
    out[0]=r->buttons; out[1]=(uint8_t)r->dx; out[2]=(uint8_t)r->dy; out[3]=(uint8_t)r->wheel;
    return BP_OK;
}
bp_result bp_hid_encode_with_id(const bp_report *r,uint8_t out[5]) {
    if(out==NULL) return BP_INVALID;
    bp_result result=bp_hid_encode(r,out+1);
    if(result==BP_OK) out[0]=BP_HID_REPORT_ID;
    return result;
}

