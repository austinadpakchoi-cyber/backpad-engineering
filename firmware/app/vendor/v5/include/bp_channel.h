#ifndef BP_CHANNEL_H
#define BP_CHANNEL_H
#include "bp_hid.h"
#define BP_CHANNEL_CREDITS 3u
/* Serialized calls. Zero means one ordered local enqueue; a nonzero result
 * remains ambiguous and is never retried. Token identifies this submit only,
 * is never reused (including across epochs), and fits callback userdata. */
typedef int (*bp_submit)(void *,const uint8_t payload[4],uint32_t epoch,uint32_t token);
typedef struct {
    bool connected,encrypted,subscribed,report_mode,suspended;
} bp_channel_state;
typedef struct {
    uint32_t token;
    uint64_t since;
    bool stamped;
} bp_channel_pending;
typedef struct {
    bp_channel_state state;
    uint32_t epoch,next_token;
    bool inflight,failed,in_submit;
    unsigned outstanding;
    /* Submit order, oldest outstanding first. Exact-token completions can
     * remove any entry; they never imply completion of another entry. */
    bp_channel_pending pending[BP_CHANNEL_CREDITS];
    uint32_t accepted,completed,stale_completions,submission_errors;
    bp_submit submit;
    void *context;
} bp_channel;
void bp_channel_init(bp_channel *,bp_submit,void *);
/* A new epoch requires previous connection submissions to be quiesced. */
bool bp_channel_open(bp_channel *,uint32_t epoch);
void bp_channel_update(bp_channel *,bp_channel_state);
bool bp_channel_ready(const bp_channel *);
void bp_channel_complete(bp_channel *,uint32_t token);
/* Called by the submit adapter immediately before handing the report to BLE.
 * Preserve each timestamp until that token completes, never reset on new send. */
void bp_channel_stamp(bp_channel *,uint32_t token,uint64_t now);
bool bp_channel_oldest(const bp_channel *,uint64_t *since);
bp_tx_result bp_channel_send(void *,const bp_report *);
#endif

