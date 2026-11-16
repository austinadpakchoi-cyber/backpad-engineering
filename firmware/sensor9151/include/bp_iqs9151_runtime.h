#ifndef BP_IQS9151_RUNTIME_H
#define BP_IQS9151_RUNTIME_H
#include "bp_iqs9151.h"
/* Zephyr binding of the IQS9151 driver. All calls come from the single sensor worker thread. */
int bp_iqs9151_init(void);                       /* RDY interrupt + full chip start-up */
int bp_iqs9151_wait_ready(unsigned timeout_ms);  /* 0 when a new RDY-low window has opened */
int bp_iqs9151_read(bp_iqs9151_frame *out, bp_iqs9151_window *w);
int bp_iqs9151_write_verify(uint16_t reg, const uint8_t *data, size_t n);
int bp_iqs9151_read_reg(uint16_t reg, uint8_t *data, size_t n);
int bp_iqs9151_reati(void);
void bp_iqs9151_cancel(void);
int bp_iqs9151_recover_bus(void);
const bp_iqs9151_bus *bp_iqs9151_bus_state(void);
uint32_t bp_iqs9151_force_requests(void);
#endif

