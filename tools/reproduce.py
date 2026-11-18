"""Build the historical C algorithms and check one real capture + synthetic tails.

Python 3.10+ and a C11 compiler are required; no pip packages, SDK or hardware.
All generated files go to --out. Source files are never modified.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
from zipfile import ZipFile

ROOT = Path(__file__).resolve().parents[1]
REPLAY_FILES = (
    "app/src/bp_touch_policy.c", "app/include/bp_touch_policy.h",
    "app/vendor/v5/src/bp_pointer.c", "app/vendor/v5/include/bp_pointer.h",
)

def digest(data):
    return hashlib.sha256(data).hexdigest()

def require(condition, message):
    if not condition:
        raise RuntimeError(message)

def check_release_tail_buttons(label, rows):
    """A move, stop, and lift must produce neither a click nor a stuck button."""
    for row in rows:
        case = (f"{label} release tail (cadence={row['cadence']} ms, "
                f"tail={row['tail_um']} um/{row['tail_ms']} ms)")
        require(int(row["left"]) == 0, f"{case}: unexpected left-button press")
        require(int(row["final_buttons"]) == 0, f"{case}: button still pressed after lift")

def apply_diff(original, patch):
    """Apply our small, context-checked unified diffs without requiring git."""
    src = original.splitlines(True)
    lines = patch.splitlines(True)
    result, pos, i = [], 0, 0
    while i < len(lines):
        match = re.match(r"@@ -(\d+)(?:,\d+)? \+\d+(?:,\d+)? @@", lines[i])
        if not match:
            i += 1
            continue
        start = int(match[1]) - 1
        require(start >= pos, "overlapping diff hunks")
        result.extend(src[pos:start])
        pos, i = start, i + 1
        while i < len(lines) and not lines[i].startswith("@@"):
            line = lines[i]
            if line[:1] in (" ", "-"):
                require(pos < len(src) and src[pos] == line[1:], "diff context changed")
                if line[0] == " ":
                    result.append(src[pos])
                pos += 1
            elif line.startswith("+"):
                result.append(line[1:])
            else:
                raise RuntimeError("unsupported diff line")
            i += 1
    result.extend(src[pos:])
    return "".join(result)

def load_r2_source():
    """Read the SOURCE.json export using exact copies of changed files."""
    source = json.loads((ROOT/"SOURCE.json").read_text(encoding="utf-8"))
    entries = {item["file"]: item["normalized_export_sha256"] for item in source["files"]}
    archive = ROOT/"samples/variants/r2-export-overrides.zip"
    with ZipFile(archive) as overrides:
        names = overrides.namelist()
        require(len(names) == len(set(names)) and set(names) <= entries.keys(),
                "Unexpected R2 source override")
        files = {}
        for rel, expected in entries.items():
            data = overrides.read(rel) if rel in names else (ROOT/rel).read_bytes().replace(b"\r\n", b"\n")
            require(digest(data) == expected, "Archived R2 source differs from SOURCE.json: "+rel)
            files[rel.removeprefix("firmware/")] = data
    return source, files

def stage_replay_sources(out, archived=None, changes=()):
    """Select matching policy and pointer source/header pairs for one build."""
    out.mkdir()
    differences = {item["file"]: item for item in changes}
    for rel in REPLAY_FILES:
        data = archived[rel] if archived is not None else (ROOT/"firmware"/rel).read_bytes()
        if rel in differences:
            change = differences[rel]
            if change["patch"]:
                data = apply_diff(data.decode("utf-8"), (ROOT/change["patch"]).read_text(encoding="utf-8")).encode()
            require(digest(data) == change["normalized_sha256"], "Historical reconstruction mismatch: "+rel)
        (out/Path(rel).name).write_bytes(data)

def compiler_environment(out):
    env = os.environ.copy()
    for key in ("TMP", "TEMP", "TMPDIR"):
        env[key] = str(out)
    if os.name != "nt":
        cc = os.environ.get("CC") or shutil.which("cc") or shutil.which("clang") or shutil.which("gcc")
        require(bool(cc), "Install a C11 compiler or set CC.")
        return cc, env, False
    if not shutil.which("cl"):
        locator = Path(os.environ.get("ProgramFiles(x86)", "")) / "Microsoft Visual Studio/Installer/vswhere.exe"
        require(locator.exists(), "Run in an x64 Native Tools prompt with cl, or install MSVC Build Tools.")
        install = subprocess.check_output([str(locator), "-latest", "-products", "*", "-requires",
                    "Microsoft.VisualStudio.Component.VC.Tools.x86.x64", "-property", "installationPath"],
                    text=True).strip()
        vcvars = Path(install) / "VC/Auxiliary/Build/vcvars64.bat"
        require(vcvars.exists(), "MSVC x64 tools were not found.")
        # Parse environment in memory only; never write or display credentials from it.
        wrapper = out/"compiler-env.cmd"
        wrapper.write_text('@echo off\ncall "'+str(vcvars)+'" >nul\nif errorlevel 1 exit /b 1\nset\n',
                           encoding="utf-8")
        setup = subprocess.run(["cmd.exe", "/u", "/d", "/c", str(wrapper)],
                               capture_output=True, text=True, encoding="utf-16-le",
                               errors="replace", env=env, check=True)
        for line in setup.stdout.splitlines():
            if "=" in line and not line.startswith("="):
                k, v = line.split("=", 1)
                env[k.upper()] = v
    compiler = shutil.which("cl", path=env.get("PATH"))
    require(bool(compiler), "cl.exe was not found in the initialized compiler environment.")
    return compiler, env, True

def compile_program(cc, env, msvc, out, variant_dir, harness, name):
    firmware = ROOT / "firmware"
    includes = [variant_dir, firmware/"app/include", firmware/"app/vendor/v5/include",
                firmware/"sensor9151/include"]
    pointer = variant_dir/"bp_pointer.c"
    if not pointer.is_file():
        pointer = firmware/"app/vendor/v5/src/bp_pointer.c"
    sources = [ROOT/"tools"/harness, pointer]
    if Path(harness).name == "replay.c":
        sources.append(variant_dir/"bp_touch_policy.c")
    flags = ["BP_PHYSICAL_X_UM=54000", "BP_PHYSICAL_Y_UM=49500", "BP_SECONDARY_RELATIVE_PERCENT=20"]
    dest = out/name
    dest.mkdir()
    exe = dest/("check.exe" if msvc else "check")
    if msvc:
        cmd = [cc, "/nologo", "/std:c11", "/utf-8", "/W3", "/O2", "/MT",
               *["/D"+f for f in flags], *["/I"+str(p) for p in includes],
               *map(str, sources), "/Fe:"+str(exe), "/link", "/INCREMENTAL:NO"]
    else:
        cmd = [cc, "-std=c11", "-O2", "-Wall", *["-D"+f for f in flags],
               *["-I"+str(p) for p in includes], *map(str, sources), "-lm", "-o", str(exe)]
    run = subprocess.run(cmd, cwd=dest, env=env, capture_output=True, text=True, errors="replace")
    (dest/"compiler.log").write_text(run.stdout+run.stderr, encoding="utf-8")
    require(run.returncode == 0, "C build failed: "+str(dest/"compiler.log"))
    return exe

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, default=ROOT/".build/replay")
    args = parser.parse_args()
    out = args.out.resolve()
    require(not out.exists() or not any(out.iterdir()), "--out must be new or empty; existing results are not overwritten.")
    out.mkdir(parents=True, exist_ok=True)
    sample = ROOT/"samples/pressure-drop"
    metadata = json.loads((sample/"metadata.json").read_text(encoding="utf-8"))
    for name, expected in metadata["sha256"].items():
        require(digest((sample/name).read_bytes()) == expected, "Sample changed: "+name)
    source, archived = load_r2_source()
    variants = json.loads((ROOT/"samples/variants/manifest.json").read_text())
    cc, env, msvc = compiler_environment(out)
    results = {"kind": "current local execution", "hardware_used": False,
               "historical_r2_source_id": source["historical_full_source_id"],
               "compiler_family": "MSVC" if msvc else "C11 cc",
               "input_kind": "real normalized 1600-frame sensor capture",
               "synthetic_scope": "18 generated stopped-finger/pressure-tail cases, not real captures",
               "real_capture": {}, "release_tail": {}, "executables": {}}
    tail_rows = {}
    r2_events = None
    for label in ("v15", "r1", "r2", "current"):
        variant = out/label
        stage_replay_sources(variant, None if label == "current" else archived,
                             variants.get(label, {}).get("changes", ()))
        exe = compile_program(cc, env, msvc, out, variant, "replay.c", label+"-capture")
        args = [str(exe), *[str(sample/n) for n in ("capture.bin", "touch.bin", "filter.bin", "gesture.bin")]]
        run = subprocess.run(args, capture_output=True, text=True, env=env, check=True)
        events = [json.loads(line[7:]) for line in run.stderr.splitlines() if line.startswith("REPORT ")]
        if label == "r2":
            r2_events = events
        elif label == "current":
            r2_controls = [e for e in r2_events if e["buttons"] or e["wheel"]]
            current_controls = [e for e in events if e["buttons"] or e["wheel"]]
            require(current_controls == r2_controls, "Current replay changed button or wheel reports on the public fixture")
            results["current_hid_event_comparison"] = {"matches_r2": events == r2_events,
                "current_events": len(events), "r2_events": len(r2_events)}
            if events == r2_events:
                results["current_matches_r2_hid_events"] = len(events)
        start, end = metadata["motion_window_ms"]
        bstart, bend = metadata["button_window_ms"]
        motion = [e for e in events if start <= e["ms"] <= end and (e["dx"] or e["dy"])]
        pressed = [e for e in events if bstart <= e["ms"] <= bend and e["buttons"]]
        result = {"motion": len(motion), "pressed": len(pressed), "final_buttons": events[-1]["buttons"],
                  "first_report_after_window_start_ms": motion[0]["ms"]-start if motion else None}
        results["real_capture"][label] = result
        if label in metadata["expected"]:
            for field, expected in metadata["expected"][label].items():
                require(result[field] == expected, f"{label}: unexpected {field}: {result[field]}")
        if label == "current":
            require(result["motion"] > 0 and result["pressed"] == 0,
                    "Current replay did not move without a press in the public failure window")
        require(result["final_buttons"] == 0, label+": stuck button")
        # Persist only compact results, not 1600-row logs from every version.
        tail = compile_program(cc, env, msvc, out, variant, "release_tail.c", label+"-tail")
        run = subprocess.run([str(tail), str(ROOT/"samples/release-tail/gesture.bin")],
                             env=env, capture_output=True, text=True, check=True)
        rows = list(csv.DictReader(io.StringIO(run.stdout)))
        require(len(rows) == 18, "Unexpected synthetic case count")
        check_release_tail_buttons(label, rows)
        tail_rows[label] = rows
        results["release_tail"][label] = rows
        results["executables"][label] = {"capture": str(exe.relative_to(out)), "tail": str(tail.relative_to(out))}
    regressions = {}
    for label in ("r1", "r2", "current"):
        n = 0
        for baseline, row in zip(tail_rows["v15"], tail_rows[label]):
            require(all(baseline[k] == row[k] for k in ("cadence","tail_um","tail_ms")), "Case order changed")
            if float(row["hid_tail_counts"]) > float(baseline["hid_tail_counts"])+1.0:
                n += 1
        regressions[label] = n
    require(regressions["r1"] == 6, "Historical R1 counterexample did not reproduce")
    require(regressions["r2"] == 0, "R2 regressed >1 HID count in a stopped-finger case")
    require(regressions["current"] == 0, "Current replay regressed >1 HID count in a stopped-finger case")
    results["tail_regressions_over_v15_gt_1_count"] = regressions
    results["passed"] = True
    (out/"result.json").write_text(json.dumps(results, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"passed": True, "real_capture": results["real_capture"],
                      "current_hid_event_comparison": results["current_hid_event_comparison"],
                      "synthetic_cases": 18, "tail_regressions": regressions}, indent=2))

if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, subprocess.CalledProcessError) as error:
        print(str(error), file=sys.stderr)
        sys.exit(1)
