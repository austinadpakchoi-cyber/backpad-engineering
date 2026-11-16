#include "bp_channel.h"
#include <string.h>
void bp_channel_init(bp_channel *c,bp_submit submit,void *context) {
    if(c==NULL) return;
    memset(c,0,sizeof(*c)); c->submit=submit; c->context=context;
}
bool bp_channel_open(bp_channel *c,uint32_t epoch) {
    if(c==NULL || c->in_submit || c->submit==NULL || epoch==0u || epoch<=c->epoch) return false;
    c->epoch=epoch; c->failed=false; c->inflight=false; c->outstanding=0u;
    memset(c->pending,0,sizeof(c->pending));
    /* next_token is intentionally retained across connection epochs. */
    memset(&c->state,0,sizeof(c->state)); c->state.connected=true; c->state.report_mode=true;
    return true;
}
void bp_channel_update(bp_channel *c,bp_channel_state state) {
    if(c!=NULL && !c->in_submit) c->state=state;
}
bool bp_channel_ready(const bp_channel *c) {
    return c!=NULL && c->submit!=NULL && c->epoch!=0u && !c->failed && c->state.connected &&
        c->state.encrypted && c->state.subscribed && c->state.report_mode && !c->state.suspended;
}
void bp_channel_complete(bp_channel *c,uint32_t token) {
    if(c==NULL) return;
    for(unsigned i=0u;i<c->outstanding;++i) {
        if(c->pending[i].token!=token) continue;
        --c->outstanding;
        memmove(&c->pending[i],&c->pending[i+1u],
                (c->outstanding-i)*sizeof(c->pending[0]));
        memset(&c->pending[c->outstanding],0,sizeof(c->pending[0]));
        c->inflight=c->outstanding!=0u; ++c->completed; return;
    }
    ++c->stale_completions;
}
void bp_channel_stamp(bp_channel *c,uint32_t token,uint64_t now) {
    if(c==NULL) return;
    for(unsigned i=0u;i<c->outstanding;++i) {
        if(c->pending[i].token==token && !c->pending[i].stamped) {
            c->pending[i].since=now; c->pending[i].stamped=true; return;
        }
    }
}
bool bp_channel_oldest(const bp_channel *c,uint64_t *since) {
    if(c==NULL || since==NULL || !c->outstanding || !c->pending[0].stamped) return false;
    *since=c->pending[0].since; return true;
}
bp_tx_result bp_channel_send(void *context,const bp_report *report) {
    bp_channel *c=context;
    if(c==NULL || report==NULL || c->in_submit) return BP_TX_ERROR;
    if(report->link_epoch!=c->epoch || !bp_channel_ready(c)) return BP_TX_DOWN;
    if(c->outstanding>=BP_CHANNEL_CREDITS) return BP_TX_BUSY; /* never handed to stack */
    uint8_t payload[4];
    if(bp_hid_encode(report,payload)!=BP_OK) return BP_TX_ERROR;
    /* Never wrap/reuse userdata: late callbacks must not match a newer send. */
    if(c->next_token==UINT32_MAX) { c->failed=true; return BP_TX_ERROR; }
    uint32_t token=++c->next_token;
    c->pending[c->outstanding++]=(bp_channel_pending){.token=token};
    c->inflight=true; c->in_submit=true;
    /* Register before submit: even a synchronous completion finds this token. */
    int error=c->submit(c->context,payload,c->epoch,token); c->in_submit=false;
    if(error!=0) { c->failed=true; ++c->submission_errors; return BP_TX_UNKNOWN; }
    ++c->accepted; return BP_TX_ACCEPTED;
}

