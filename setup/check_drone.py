"""connect to a drone and check everything before flying. doesnt take off

decks, battery, lighthouse base stations, and whether the position holds still.
leave the drone sitting in the flight area while it runs.

python setup/check_drone.py [--uri radio://...]
"""

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

from common.paths import CACHE_DIR

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"
SECONDS = 3.0            # how long to watch the position
MIN_VBAT = 3.8           # V, below this charge it
MAX_JITTER_MM = 5.0      # position std while sitting still


def ok(good, text):
    print(("  ok  " if good else "  !!  ") + text)
    return good


def start_log(cf, name, variables, on_data):
    # skip variables this firmware doesnt have
    present = [v for v in variables if cf.log.toc.get_element_by_complete_name(v) is not None]
    if not present:
        return None
    conf = LogConfig(name=name, period_in_ms=50)
    for v in present:
        conf.add_variable(v)
    cf.log.add_config(conf)
    conf.data_received_cb.add_callback(lambda _t, data, _c: on_data(data))
    conf.start()
    return conf


def bits(mask):
    return [i for i in range(16) if mask >> i & 1]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()

    cflib.crtp.init_drivers()
    print(f"connecting to {args.uri} ...")
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(CACHE_DIR))) as scf:
        scf.wait_for_params()
        cf = scf.cf
        all_good = True

        print("\ndecks")
        decks = [name[2:] for name in cf.param.toc.toc.get("deck", {})
                 if name.startswith("bc") and int(cf.param.get_value(f"deck.{name}"))]
        print(f"      found: {', '.join(decks) or 'none'}")
        all_good &= ok(any(d.startswith("Lighthouse") for d in decks), "lighthouse deck")

        samples = []
        latest = {}

        def on_data(data):
            latest.update(data)
            if "stateEstimate.x" in data:
                samples.append([data["stateEstimate.x"], data["stateEstimate.y"], data["stateEstimate.z"]])

        logs = [
            start_log(cf, "check_a", ["pm.vbat", "stateEstimate.x", "stateEstimate.y", "stateEstimate.z"], on_data),
            start_log(cf, "check_b", ["lighthouse.bsReceive", "lighthouse.bsCalVal", "lighthouse.bsGeoVal"], on_data),
        ]
        time.sleep(SECONDS)
        for log in logs:
            if log is not None:
                log.stop()

        print("\nbattery")
        vbat = latest.get("pm.vbat")
        if vbat is not None:
            all_good &= ok(vbat >= MIN_VBAT, f"{vbat:.2f} V")

        print("\nlighthouse")
        if "lighthouse.bsReceive" in latest:
            receive = bits(int(latest["lighthouse.bsReceive"]))
            cal = bits(int(latest.get("lighthouse.bsCalVal", 0)))
            geo = bits(int(latest.get("lighthouse.bsGeoVal", 0)))
            for bs in sorted(set(receive) | set(cal) | set(geo)):
                good = bs in receive and bs in cal and bs in geo
                all_good &= ok(good, f"base station {bs + 1}: receiving {bs in receive}, "
                                     f"calibration {bs in cal}, geometry {bs in geo}")
            all_good &= ok(len(receive) >= 2, f"{len(receive)} base stations received (want 2+)")
        else:
            all_good &= ok(False, "no lighthouse log variables (deck missing or old firmware?)")

        print("\nposition")
        if samples:
            p = np.array(samples)
            jitter = p.std(axis=0).max() * 1000
            x, y, z = p.mean(axis=0)
            print(f"      at ({x:+.3f}, {y:+.3f}, {z:+.3f}) m")
            all_good &= ok(jitter <= MAX_JITTER_MM, f"jitter {jitter:.1f} mm")
        else:
            all_good &= ok(False, "no position estimate")

    print("\nlooks good, ready to fly" if all_good else "\nfix the !! lines first (setup/README.md has fixes)")


if __name__ == "__main__":
    main()
