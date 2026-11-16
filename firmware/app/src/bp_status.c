#include "bp_status.h"
#include <stdatomic.h>
bool bp_status_publish_due(volatile bp_test_status *published,
                           const bp_test_status *working, uint64_t now,
                           uint64_t *next_due, bool force) {
    if(!force && now<*next_due) return false;
    uint32_t next=(published->snapshot_seq & ~1u)+2u;
    if(!next) next=2u;
    bp_test_status snapshot=*working;
    snapshot.snapshot_seq=next|1u;
    published->snapshot_seq=next|1u;
    atomic_thread_fence(memory_order_seq_cst);
    *published=snapshot;
    atomic_thread_fence(memory_order_seq_cst);
    published->snapshot_seq=next;
    *next_due=now+BP_STATUS_PERIOD_MS;
    return true;
}

