"""Generate a build ID from this export plus externally supplied sensor settings."""
from pathlib import Path
import argparse
import hashlib

p = argparse.ArgumentParser()
p.add_argument("--settings", type=Path, required=True)
p.add_argument("--out", type=Path, required=True)
args = p.parse_args()
root = Path(__file__).resolve().parents[1]/"firmware"
h = hashlib.sha256()
files = [(f.relative_to(root).as_posix(), f) for f in sorted(root.rglob("*")) if f.is_file()]
files.append(("external/iqs9151_settings.c", args.settings))
for name, f in files:
    data = f.read_bytes().replace(b"\r\n", b"\n")
    h.update(name.encode()+b"\0"+str(len(data)).encode()+b"\0"+data)
args.out.mkdir(parents=True, exist_ok=True)
(args.out/"build_id.h").write_text('#define BP_BUILD_SHA "'+h.hexdigest()+'"\n', encoding="utf-8")
print(h.hexdigest())
