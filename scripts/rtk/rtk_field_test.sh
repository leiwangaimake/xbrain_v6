#!/usr/bin/env bash
#
# Copyright (c) 2026 Hachist Robotics
# Author: wanglei@hachist.com
# 上海哈船智能船舶技术有限公司
# File: rtk_field_test.sh
# Brief: Outdoor RTK test -- raw NMEA check, then rtk_driver with RTCM, fix_type timeline
#
# Description:
# The field procedure of ros2_ws/sensor/README.md S5.3, as one command on the
# ORIN. Two phases:
#   1) raw serial capture (rtk_driver must NOT be running: the port has one
#      owner) -- refuses to go on when the UM982 sends nothing, because then the
#      problem is power / wiring, not RTK;
#   2) the real rtk_driver binary with configs/rtk_driver.yaml (mode ntrip: the
#      file as is, LAN off, Qianxun on; mode lan: LAN pointed at HOST:PORT, NTRIP
#      left as configured), an RT router if none is up, and a subscriber that
#      logs every rt/gnss/fix fix_type change with its elapsed time and writes a
#      1 Hz CSV. The summary gives the time to first rtk_float / rtk_fixed
#      (M-RTCM-3, 10 M-2) and the share of time fixed.
# Everything lands in /home/nvidia/rtk_field/<timestamp>/ (not /tmp: a reboot
# in the field must not take the evidence with it).
#
# The resolved config is a copy of the source file, not the freeze product: on
# this bench the full freeze stops on unrelated common.* placeholders, and
# rtk_driver.yaml carries no ${common.*} reference to expand.
#
# usage: scripts/rtk/rtk_field_test.sh ntrip [seconds]
#        scripts/rtk/rtk_field_test.sh lan HOST:PORT [seconds]
# RTK_FIELD_SKIP_RAW=1 skips phase 1 -- bench check of phase 2 only, never in the field.

set -u
ROOT=/opt/xbrain_v6
MODE=${1:-}
if [ "$MODE" = "lan" ]; then
  LAN_EP=${2:?usage: rtk_field_test.sh lan HOST:PORT [seconds]}
  DURATION=${3:-600}
elif [ "$MODE" = "ntrip" ]; then
  DURATION=${2:-600}
else
  echo "usage: rtk_field_test.sh ntrip [seconds] | lan HOST:PORT [seconds]" >&2
  exit 2
fi
OUT=/home/nvidia/rtk_field/$(date +%Y%m%d-%H%M%S)-$MODE
mkdir -p "$OUT/resolved"
BIN=$ROOT/ros2_ws/sensor/build/rtk_driver
PORT_DEV=$(sed -n 's/^  port: "\(.*\)".*/\1/p' $ROOT/configs/rtk_driver.yaml | head -1)
RID=$(sed -n 's/^XBRAIN_ROBOT_ID=//p' $ROOT/deploy/env/robot.env | head -1)
echo "output: $OUT  serial: $PORT_DEV  rid: $RID  duration: ${DURATION}s"

if pgrep -x rtk_driver > /dev/null; then
  echo "ERROR: rtk_driver is already running (it owns $PORT_DEV); stop it first" >&2
  exit 1
fi
[ -x "$BIN" ] || { echo "ERROR: $BIN not built" >&2; exit 1; }

# ---- phase 1: raw NMEA -------------------------------------------------------
echo "== phase 1: raw serial capture (10 s)"
stty -F "$PORT_DEV" 115200 raw -echo || exit 1
timeout 10 cat "$PORT_DEV" > "$OUT/raw.bin"
BYTES=$(wc -c < "$OUT/raw.bin")
GGA=$(grep -a -c 'GGA,' "$OUT/raw.bin")
echo "bytes=$BYTES gga_lines=$GGA"
if [ "$BYTES" -eq 0 ] && [ "${RTK_FIELD_SKIP_RAW:-0}" != "1" ]; then
  echo "ERROR: the UM982 sends nothing -- check its power and wiring before RTK" >&2
  exit 1
fi
grep -a 'GGA,' "$OUT/raw.bin" | tail -1 | awk -F, '{print "last GGA: quality=" $7 " sats=" $8 " hdop=" $9 " diff_age=" $14}'

# ---- phase 2: rtk_driver with RTCM ------------------------------------------
echo "== phase 2: rtk_driver + RTCM ($MODE)"
python3 - "$ROOT/configs/rtk_driver.yaml" "$OUT/resolved/rtk_driver.yaml" "$MODE" "${LAN_EP:-}" <<'PY'
import re, sys
src, dst, mode, ep = sys.argv[1:5]
s = open(src, encoding="utf-8").read()
if mode == "lan":
    host, port = ep.rsplit(":", 1)
    i = s.index("  lan:")
    part = s[i:]
    part = part.replace("enabled: false", "enabled: true", 1)
    part = re.sub(r"host: \S+", 'host: "%s"' % host, part, count=1)
    part = re.sub(r"port: \S+", "port: %s" % port, part, count=1)
    s = s[:i] + part
open(dst, "w", encoding="utf-8").write(s)
PY
grep -n -A2 "^  lan:\|^  ntrip:" "$OUT/resolved/rtk_driver.yaml" | grep -v "#  " | head -6

ZPID=""
if ! ss -ltn | grep -q '127.0.0.1:7449'; then
  /usr/local/bin/zenohd --config $ROOT/deploy/zenoh/zenohd-rt.json5 > "$OUT/zenohd.log" 2>&1 &
  ZPID=$!
  sleep 2
fi
XBRAIN_ROBOT_ID=$RID XBRAIN_RESOLVED_DIR=$OUT/resolved LD_LIBRARY_PATH=/usr/local/lib \
  timeout "$DURATION" "$BIN" > "$OUT/rtk_driver.log" 2>&1 &
RPID=$!
python3 - "$RID" "$DURATION" "$OUT" <<'PY'
import json, sys, time
import zenoh
rid, duration, out = sys.argv[1], float(sys.argv[2]), sys.argv[3]
conf = zenoh.Config()
conf.insert_json5("mode", '"client"')
conf.insert_json5("connect/endpoints", '["tcp/127.0.0.1:7449"]')
s = zenoh.open(conf)
t0 = time.monotonic()
state = {"last": None, "first": {}, "secs": {}, "last_sec": -1, "row": None}
csv = open(out + "/fix.csv", "w")
csv.write("t_s,fix_type,sats,hdop,age_s,cov_h_m,lat,lon\n")
def cb(sample):
    try:
        d = json.loads(bytes(sample.payload.to_bytes()).decode())["data"]
    except Exception:
        return
    t = time.monotonic() - t0
    ft = d.get("fix_type")
    if ft != state["last"]:
        print("%7.1f s  fix_type %s -> %s  sats=%s age_s=%s cov_h_m=%s" %
              (t, state["last"], ft, d.get("sats"), d.get("age_s"), d.get("cov_h_m")), flush=True)
        state["last"] = ft
        state["first"].setdefault(ft, t)
    sec = int(t)
    if sec != state["last_sec"]:
        state["last_sec"] = sec
        state["secs"][ft] = state["secs"].get(ft, 0) + 1
        csv.write("%d,%s,%s,%s,%s,%s,%s,%s\n" % (sec, ft, d.get("sats"), d.get("hdop"),
                  d.get("age_s"), d.get("cov_h_m"), d.get("lat"), d.get("lon")))
        csv.flush()
sub = s.declare_subscriber("xbrain/%s/rt/gnss/fix" % rid, cb)
time.sleep(duration + 1)
total = sum(state["secs"].values()) or 1
print("== summary over %d s" % total)
for k in ("single", "dgps", "rtk_float", "rtk_fixed"):
    if k in state["first"]:
        print("  first %-9s at %7.1f s" % (k, state["first"][k]))
for k, v in sorted(state["secs"].items()):
    print("  %-9s %5.1f %% of the time" % (k, 100.0 * v / total))
s.close()
PY
wait $RPID
[ -n "$ZPID" ] && kill $ZPID 2>/dev/null
echo "== rtk_driver rtcm log (last lines)"
grep -E "rtcm" "$OUT/rtk_driver.log" | grep -v "active=" | tail -15
grep "rtcm active" "$OUT/rtk_driver.log" | tail -1
echo "evidence: $OUT"
