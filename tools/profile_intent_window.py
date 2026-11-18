"""Count bytes passed to memmove at the intent-window eviction site during real replay."""
import json
import subprocess
import tempfile
from pathlib import Path

import reproduce

repo = reproduce.ROOT

current = (repo / "firmware/app/src/bp_touch_policy.c").read_text(encoding="utf-8")
patch = (repo / "samples/variants/r2-bp_touch_policy.c.patch").read_text()
r2 = reproduce.apply_diff(current, patch)
sample = repo / "samples/pressure-drop"
args = [str(sample / f) for f in ("capture.bin", "touch.bin", "filter.bin", "gesture.bin")]
results = {}
with tempfile.TemporaryDirectory(dir=repo) as temp:
    out = Path(temp)
    cc, env, msvc = reproduce.compiler_environment(out)
    for label, code in (("r2", r2), ("current", current)):
        start = code.index("static void intent_observe(")
        stop = code.index("static bool lift_motion_observe(", start)
        intent = code[start:stop]
        assert intent.count("memmove(") == 1
        wrapper = """#include <stdio.h>
static void window_copy(void *out, const void *in, size_t n) {
    fprintf(stderr, "WINDOW_COPY %zu\\n", n);
    memmove(out, in, n);
}
"""
        instrumented = code[:start] + wrapper + intent.replace("memmove(", "window_copy(") + code[stop:]
        variant = out / label
        variant.mkdir()
        (variant / "bp_touch_policy.c").write_text(instrumented, encoding="utf-8")
        exe = reproduce.compile_program(cc, env, msvc, out, variant, "replay.c", label + "-capture")
        run = subprocess.run([str(exe), *args], check=True, env=env, capture_output=True, text=True)
        sizes = [int(row.split()[1]) for row in run.stderr.splitlines() if row.startswith("WINDOW_COPY ")]
        results[label] = {"calls": len(sizes), "total_bytes": sum(sizes)}
print(json.dumps(results, indent=2))
