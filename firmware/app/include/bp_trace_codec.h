#ifndef BP_TRACE_CODEC_H
#define BP_TRACE_CODEC_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#define BP_TRACE_WORDS 18u
#define BP_TRACE_BYTES 160000u
typedef struct { uint16_t previous[BP_TRACE_WORDS]; uint32_t used; } bp_trace_encoder;
static inline void bp_trace_u32(volatile uint8_t *p,uint32_t v) {
    for(unsigned j=0;j<4u;j++)p[j]=(uint8_t)(v>>(8u*j));
}
/* Lossless word-delta record: ms32,sequence32,changed_bitmap32, then LE16
 * values in bit order. Every sensor frame is kept, including no-touch frames.
 * Capacity failure changes neither encoder nor output; callers must fail the
 * capture rather than claim a complete trace. Session is fixed in header. */
static inline bool bp_trace_append(bp_trace_encoder *e,volatile uint8_t *out,size_t cap,
                                   uint32_t ms,uint32_t seq,const uint8_t xy[36]) {
    uint32_t mask=0;size_t need=12u;uint16_t words[BP_TRACE_WORDS];
    for(unsigned j=0;j<BP_TRACE_WORDS;j++) {
        words[j]=(uint16_t)((uint16_t)xy[2u*j]|((uint16_t)xy[2u*j+1u]<<8));
        if(words[j]!=e->previous[j]){mask|=1u<<j;need+=2u;}
    }
    if(e->used>cap || need>cap-e->used)return false;
    volatile uint8_t *p=out+e->used;
    bp_trace_u32(p,ms);bp_trace_u32(p+4,seq);bp_trace_u32(p+8,mask);
    size_t pos=12u;
    for(unsigned j=0;j<BP_TRACE_WORDS;j++)if(mask&(1u<<j)) {
        p[pos++]=(uint8_t)words[j];p[pos++]=(uint8_t)(words[j]>>8);
    }
    for(unsigned j=0;j<BP_TRACE_WORDS;j++)e->previous[j]=words[j];
    e->used+=(uint32_t)need;return true;
}
#endif
