#ifndef BP_BOND_BUTTON_H
#define BP_BOND_BUTTON_H
#include <stdbool.h>
#include <stdint.h>
/* Physical SW1 only. Clear on debounced RELEASE after six seconds; a short
 * tap, read error, boot with the button held, or bounce never clears bonds. */
typedef struct {
    bool raw, held, armed, initialized;
    uint64_t changed, since;
} bp_bond_button;
static inline bool bp_bond_button_poll(bp_bond_button *b, int value, uint64_t now) {
    if(value<0) { b->initialized=false; b->armed=false; return false; }
    bool raw=value>0;
    if(!b->initialized) {
        b->initialized=true; b->raw=raw; b->held=raw; b->changed=now;
        b->armed=!raw; return false;
    }
    if(raw!=b->raw) { b->raw=raw; b->changed=now; }
    if(raw==b->held || now-b->changed<30u) return false;
    b->held=raw;
    if(raw) { b->since=b->changed; return false; }
    bool clear=b->armed && b->changed-b->since>=6000u;
    b->armed=true;
    return clear;
}
#endif

