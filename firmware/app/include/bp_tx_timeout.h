#ifndef BP_TX_TIMEOUT_H
#define BP_TX_TIMEOUT_H
#include <stdbool.h>
#include <stdint.h>

/* Apply to the oldest outstanding channel token, never the most recent send.
 * A submit may record a timestamp later than the caller's loop timestamp.
 * Never interpret unsigned subtraction underflow as elapsed time. */
static inline bool bp_tx_expired(bool inflight, uint64_t now, uint64_t since) {
    return inflight && now >= since && now - since >= 1000u;
}
#endif

