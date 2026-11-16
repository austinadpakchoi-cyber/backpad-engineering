#ifndef BP_HARDWARE_CLASS_H
#define BP_HARDWARE_CLASS_H
#include <stdint.h>
/* Portable only across the exact same schematic hardware class.
 * DEVICEID and VARIANT are observations, never substituted with an old chip ID.
 * The host binds the actual DEVICEID separately for each acceptance session. */
static inline int bp_hardware_class_supported(uint32_t part, uint32_t package,
                                             uint32_t ram_kib, uint32_t flash_kib) {
    return part == 0x52840u && package == 0x2004u &&
           ram_kib == 256u && flash_kib == 1024u;
}
#endif

