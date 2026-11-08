#!/usr/bin/env bash
# Build PX4 for the Pixhawk 6C with the "DH Bailout" flight mode compiled in.
#
# Input: a PX4-Autopilot tree prepared by the organizers' own script,
#   drone-snapshot/home/dronehacks/skybeta/px4/firmware/txdma-fix/build_fmu-v6c_v1.16.2-txdma.sh
# run with PREPARE_ONLY=1. Its HEAD is 7a39c5954139c917d21e0c5cdf6cc10449b1c011
# (v1.16.2-1-g7a39c59), the firmware drone 11's FC runs. This script adds exactly
# one commit on top of it, and nothing else:
#   src/modules/dh_bailout/                  the mode (this directory's src/)
#   boards/px4/fmu-v6c/init/rc.board_extras  starts it at boot
#   CONFIG_MODULES_DH_BAILOUT=y              in boards/px4/fmu-v6c/default.px4board,
#                                            and in boards/px4/sitl/default.px4board for SITL
#   src/modules/dh_fi_emulator/              SITL-only test publisher, enabled only in the
#                                            sitl board (the script checks it is not in the .px4)
#
# Usage:  build.sh PX4_TREE [make target ...]     (default target: px4_fmu-v6c_default)
#         OUT=dir build.sh ...                    (where the .px4 is copied; default PX4_TREE/../out)
# Re-running on a tree that already has the commit replaces it with the current overlay.
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
[ $# -ge 1 ] || { sed -n '2,20p' "$0"; exit 2; }
SRC=$(cd "$1" && pwd)
shift
TARGETS=("$@")
[ ${#TARGETS[@]} -gt 0 ] || TARGETS=(px4_fmu-v6c_default)
OUT=${OUT:-$SRC/../out}
BASE=7a39c5954139c917d21e0c5cdf6cc10449b1c011
die() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

cd "$SRC"
# Fixed identity and date, so the same overlay always gives the same commit SHA
# and with it the same git_identity (v1.16.2-2-g<sha>; 5cc214d for the 2026-11-07 overlay).
export GIT_AUTHOR_NAME="dronehacks build"
export GIT_AUTHOR_EMAIL="dronehacks-build@localhost"
export GIT_AUTHOR_DATE="2026-11-07T00:00:00+0000"
export GIT_COMMITTER_NAME="$GIT_AUTHOR_NAME"
export GIT_COMMITTER_EMAIL="$GIT_AUTHOR_EMAIL"
export GIT_COMMITTER_DATE="$GIT_AUTHOR_DATE"

if [ "$(git rev-parse HEAD)" != "$BASE" ]; then
	[ "$(git rev-parse HEAD~1 2>/dev/null)" = "$BASE" ] && [ "$(git log -1 --format=%an)" = "$GIT_AUTHOR_NAME" ] \
		|| die "HEAD is neither $BASE nor one dronehacks commit on it"
	git diff --quiet && git diff --cached --quiet || die "PX4 tree has uncommitted changes"
	git checkout -q --detach "$BASE"   # drops our previous commit's files; it stays in the reflog
fi
git diff --quiet && git diff --cached --quiet || die "PX4 tree has uncommitted changes"
[ -z "$(git status --porcelain --untracked-files=normal -- src/modules/dh_bailout boards/px4/fmu-v6c/init)" ] \
	|| die "untracked files where the overlay goes"

mkdir -p src/modules/dh_bailout src/modules/dh_fi_emulator
cp "$HERE"/src/modules/dh_bailout/* src/modules/dh_bailout/
cp "$HERE"/src/modules/dh_fi_emulator/* src/modules/dh_fi_emulator/
cp "$HERE/boards/px4/fmu-v6c/init/rc.board_extras" boards/px4/fmu-v6c/init/rc.board_extras
insert_after_dataman() {  # <board file> <config line>
	grep -qx 'CONFIG_MODULES_DATAMAN=y' "$1" || die "$1: no CONFIG_MODULES_DATAMAN=y to insert after"
	grep -qx "$2" "$1" || sed -i "/^CONFIG_MODULES_DATAMAN=y\$/a $2" "$1"
}
insert_after_dataman boards/px4/fmu-v6c/default.px4board CONFIG_MODULES_DH_BAILOUT=y
insert_after_dataman boards/px4/sitl/default.px4board CONFIG_MODULES_DH_BAILOUT=y
# Test-only flight_interface emulator: SITL only, never in the Pixhawk image.
insert_after_dataman boards/px4/sitl/default.px4board CONFIG_MODULES_DH_FI_EMULATOR=y
git add src/modules/dh_bailout src/modules/dh_fi_emulator boards/px4/fmu-v6c/init/rc.board_extras \
	boards/px4/fmu-v6c/default.px4board boards/px4/sitl/default.px4board
git commit -q -m "DroneHacks: add the DH Bailout flight mode (dh_bailout)

Registers as an external mode with commander and, while selected and armed,
brakes to rest, settles and descends until the land detector and
COM_DISARM_LAND disarm. Started from boards/px4/fmu-v6c/init/rc.board_extras.
Source: DroneHacks repo, firmware/bailout."
[ "$(git rev-parse HEAD~1)" = "$BASE" ] || die "commit is not on $BASE"
IDENT=$(git describe --exclude 'ext/*' --always --tags --dirty)
case "$IDENT" in v1.16.2-2-g*) ;; *) die "unexpected git describe: $IDENT";; esac
echo "source: $IDENT ($(git rev-parse HEAD))"

for target in "${TARGETS[@]}"; do
	make "$target" -j"$(nproc)"
done

case " ${TARGETS[*]} " in
*" px4_fmu-v6c_default "*)
	PX4=build/px4_fmu-v6c_default/px4_fmu-v6c_default.px4
	[ -f "$PX4" ] || die "no $PX4"
	/usr/bin/python3 -I - "$PX4" <<'EOF'
import base64, json, sys, zlib
d = json.load(open(sys.argv[1]))
img = zlib.decompress(base64.b64decode(d["image"]))
for k in ("summary", "board_id", "git_identity", "git_hash", "image_size", "image_maxsize"):
    print(f"{k}: {d.get(k)}")
assert d["board_id"] == 56, "not fmu-v6c"
assert d["git_identity"].startswith("v1.16.2-2-g"), d["git_identity"]
assert d["image_size"] <= d["image_maxsize"], "image does not fit"
assert b"dh_bailout" in img and b"DH Bailout" in img, "module missing from the image"
assert b"dh_fi_emulator" not in img, "SITL-only test module leaked into the Pixhawk image"
print(f"free flash: {d['image_maxsize'] - d['image_size']} bytes")
EOF
	mkdir -p "$OUT"
	NAME=px4_fmu-v6c_default-${IDENT}-dh-bailout.px4
	cp "$PX4" "$OUT/$NAME"
	(cd "$OUT" && sha256sum "$NAME" > "$NAME.sha256" && cat "$NAME.sha256")
	;;
esac
