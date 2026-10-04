#!/usr/bin/env bash
# List /dev/pts with owning process names.
set -euo pipefail

echo "self tty: $(tty 2>/dev/null || echo none)"
echo
python3 - <<'PY'
import os, collections
owners = collections.defaultdict(list)
for pid in os.listdir("/proc"):
    if not pid.isdigit():
        continue
    comm = ""
    try:
        with open(f"/proc/{pid}/comm") as f:
            comm = f.read().strip()
    except OSError:
        comm = pid
    fd_dir = f"/proc/{pid}/fd"
    try:
        for fd in os.listdir(fd_dir):
            try:
                target = os.readlink(f"{fd_dir}/{fd}")
            except OSError:
                continue
            if target.startswith("/dev/pts/"):
                owners[target].append(comm)
    except OSError:
        continue

print(f"{'path':<14} {'owners':<40} note")
names = [n for n in os.listdir("/dev/pts") if n.isdigit()]
names.sort(key=int, reverse=True)
for name in names:
    path = f"/dev/pts/{name}"
    own = owners.get(path, [])
    note = "UNOWNED" if not own else ""
    joined = ",".join(own) if own else "-"
    if any(("homework" in o or "godot" in o.lower() or "x86_64" in o) for o in own):
        note = "GAME?"
    print(f"{path:<14} {joined:<40} {note}")
PY
