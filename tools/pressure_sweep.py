"""Replay a grid of temporary strength drops in one recorded, single-finger stroke.

python -B tools/pressure_sweep.py --device cuda --out result.json
CUDA generation requires a CUDA-enabled PyTorch installation. Use --device cpu
for the portable scalar reference. Both modes run the real C policy on the host.
"""
import argparse
import json
import struct
import subprocess
import tempfile
from pathlib import Path

from reproduce import ROOT, compile_program, compiler_environment, digest, require


SAMPLE = ROOT / "samples/pressure-drop"
DEPTHS = (15, 30, 50, 70, 90)
DURATIONS = (24, 48, 72, 96, 144, 240)
CASES = [(0, 0)] + [(d, t) for d in DEPTHS for t in DURATIONS]
HEADER, FRAME, STRENGTH = 128, 48, 24


def scalar_row(values, elapsed, depth, duration):
    return [(value * (100 - depth) + 50) // 100 if 0 <= age < duration else value
            for value, age in zip(values, elapsed)]


def generate(values, elapsed, backend):
    reference = [scalar_row(values, elapsed, d, t) for d, t in CASES]
    if backend == "cpu":
        return reference, {"backend": "cpu", "implementation": "scalar integer reference"}
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("CUDA mode needs a CUDA-enabled PyTorch build; or select --device cpu.") from exc
    require(torch.cuda.is_available(), "CUDA device unavailable. Select --device cpu for a CPU-only run.")
    depths, durations = torch.tensor(CASES, dtype=torch.int64, device="cuda").T.unsqueeze(-1)
    age = torch.tensor(elapsed, dtype=torch.int64, device="cuda")
    strength = torch.tensor(values, dtype=torch.int64, device="cuda")
    attenuated = (strength * (100 - depths) + 50) // 100
    result = torch.where((age >= 0) & (age < durations), attenuated, strength)
    rows = result.cpu().tolist()
    require(rows == reference, "CUDA inputs differ from the independent scalar implementation.")
    return rows, {"backend": "cuda", "device": torch.cuda.get_device_name(result.device),
                  "torch": str(torch.__version__), "torch_cuda": torch.version.cuda,
                  "scalar_comparison_values": int(result.numel())}


def replay(exe, capture_file, env):
    args = [exe, capture_file, SAMPLE/"touch.bin", SAMPLE/"filter.bin", SAMPLE/"gesture.bin"]
    run = subprocess.run(args, capture_output=True, text=True, check=True, env=env)
    reports = [json.loads(line[7:]) for line in run.stderr.splitlines() if line.startswith("REPORT ")]
    require(bool(reports), "C replay produced no HID records.")
    return reports


def measure(events, metadata):
    start, end = metadata["motion_window_ms"]
    bstart, bend = metadata["button_window_ms"]
    movement = [e for e in events if start <= e["ms"] <= end and (e["dx"] or e["dy"])]
    return {"motion": len(movement),
            "pressed_reports": sum(1 for e in events if bstart <= e["ms"] <= bend and e["buttons"]),
            "first_motion_ms_after_start": movement[0]["ms"] - start if movement else None,
            "final_buttons": events[-1]["buttons"]}


def run(backend, scratch):
    meta = json.loads((SAMPLE/"metadata.json").read_text(encoding="utf-8"))
    for name, expected in meta["sha256"].items():
        require(digest((SAMPLE/name).read_bytes()) == expected, "Unexpected public fixture: " + name)
    capture = (SAMPLE/"capture.bin").read_bytes()
    require(len(capture) == HEADER + meta["frames"] * FRAME, "Unexpected capture layout")
    offsets = range(HEADER, len(capture), FRAME)
    elapsed = [struct.unpack_from("<I", capture, p)[0] - meta["motion_window_ms"][0] for p in offsets]
    for p, age in zip(offsets, elapsed):
        if 0 <= age < max(DURATIONS):
            x, y = struct.unpack_from("<HH", capture, p+20)
            require((capture[p+18] & 15) == 1 and x != 65535 and y != 65535,
                    "The selected perturbation interval must contain a single valid slot-0 contact.")
    original = [struct.unpack_from("<H", capture, p+STRENGTH)[0] for p in offsets]
    rows, runtime = generate(original, elapsed, backend)
    bindings = ("firmware/app/src/bp_touch_policy.c", "firmware/app/include/bp_touch_policy.h",
                "firmware/app/vendor/v5/src/bp_pointer.c", "tools/replay.c")
    source_hashes = {rel: digest((ROOT/rel).read_bytes().replace(b"\r\n", b"\n")) for rel in bindings}
    cc, env, msvc = compiler_environment(scratch)
    policy = ROOT/"firmware/app/src"
    exe = compile_program(cc, env, msvc, scratch, policy, "replay.c", "native")
    baseline = replay(exe, SAMPLE/"capture.bin", env)
    baseline_metrics = measure(baseline, meta)
    require(baseline_metrics["motion"] == meta["expected"]["r2"]["motion"] and
            baseline_metrics["pressed_reports"] == meta["expected"]["r2"]["pressed"] and
            baseline_metrics["final_buttons"] == 0, "Current candidate differs from the R2 fixture expectations.")
    cases = []
    case_file = scratch/"perturbed.bin"
    for (depth, duration), row in zip(CASES, rows):
        modified = bytearray(capture)
        changed = 0
        for p, value, before in zip(offsets, row, original):
            if value != before:
                struct.pack_into("<H", modified, p+STRENGTH, value)
                changed += 1
        case_file.write_bytes(modified)
        events = replay(exe, case_file, env)
        if depth == 0:
            require(modified == capture and events == baseline, "Identity control diverged.")
        metrics = measure(events, meta)
        cases.append({"reduction_percent": depth, "duration_ms": duration, "frames_changed": changed,
                      **metrics})
    def risk(case):
        return case["motion"] == 0 or case["pressed_reports"] != 0 or case["final_buttons"] != 0
    for rel, sha in source_hashes.items():
        require(digest((ROOT/rel).read_bytes().replace(b"\r\n", b"\n")) == sha, "Source changed during run: " + rel)
    return {"kind": "offline strength sensitivity on a single recorded stroke",
            "runtime": runtime, "native_compiler": "MSVC" if msvc else "C11 cc",
            "algorithm": "current working-tree candidate, compiled from C; control compared with R2 fixture expectations",
            "source_sha256": source_hashes,
            "input_capture_sha256": meta["sha256"]["capture.bin"], "frames": meta["frames"],
            "perturbation": "Rectangular reduction of slot-0 strength only, starting at motion_window_ms[0], excluding the duration endpoint; integer round-half-up. Coordinates, area and timing are preserved.",
            "motion_window_ms": meta["motion_window_ms"], "button_window_ms": meta["button_window_ms"],
            "control": {"identical_capture_and_all_hid_reports": True, "report_count": len(baseline), **baseline_metrics},
            "synthetic_cases": len(cases)-1,
            "flag_criterion": "Zero motion in the motion window, any pressed report in the button window, or a nonzero final button state. One source capture does not establish device accuracy.",
            "flagged_cases": [[c["reduction_percent"], c["duration_ms"]] for c in cases[1:] if risk(c)],
            "cases": cases}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=["cuda", "cpu"], default="cuda")
    parser.add_argument("--out", type=Path, help="Compact JSON output. The default prints JSON to stdout.")
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix=".pressure-sweep-", dir=ROOT) as scratch:
        result = run(args.device, Path(scratch))
    text = json.dumps(result, indent=2) + "\n"
    if args.out:
        args.out.write_text(text, encoding="utf-8")
        print(f"{len(result['flagged_cases'])}/{result['synthetic_cases']} flagged; {args.out}")
    else:
        print(text, end="")


if __name__ == "__main__":
    main()
