"""save / load the lighthouse config (base station geometry + calibration)

save: drone -> setup/lighthouse_configs/<file>
load: setup/lighthouse_configs/<file> -> drone (stored on it, survives power off)

python setup/lighthouse_config.py save lab.yaml [--uri radio://...]
python setup/lighthouse_config.py load lab.yaml [--uri radio://...]
"""

import argparse
import sys
import threading
from pathlib import Path

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.mem import LighthouseMemHelper
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.localization.lighthouse_config_manager import LighthouseConfigFileManager, LighthouseConfigWriter

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.paths import CACHE_DIR

# settings
URI = "radio://0/80/2M/E7E7E7E7E9"
CONFIG_DIR = Path(__file__).resolve().parent / "lighthouse_configs"
TIMEOUT = 10.0           # s


def wait(event, what):
    if not event.wait(TIMEOUT):
        sys.exit(f"timed out {what}")


def read(read_fn):
    done = threading.Event()
    result = {}

    def cb(data):
        result.update(data)
        done.set()

    read_fn(cb)
    wait(done, "reading from the drone")
    return result


def save(cf, path):
    helper = LighthouseMemHelper(cf)
    geos = read(helper.read_all_geos)
    calibs = read(helper.read_all_calibs)
    system_type = int(cf.param.get_value("lighthouse.systemType"))

    geos = {ch: g for ch, g in geos.items() if g.valid}
    calibs = {ch: c for ch, c in calibs.items() if c.valid}
    if not geos:
        sys.exit("drone has no geometry yet, do the setup in cfclient first")
    LighthouseConfigFileManager.write(str(path), geos, calibs, system_type)
    print(f"saved {len(geos)} base stations to {path}")


def load(cf, path):
    if not path.exists():
        sys.exit(f"no such file {path}")
    done = threading.Event()
    result = {}

    def cb(success):
        result["ok"] = success
        done.set()

    LighthouseConfigWriter(cf).write_and_store_config_from_file(cb, str(path))
    wait(done, "writing to the drone")
    print(f"loaded {path.name}" if result["ok"] else "write failed, try again")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("action", choices=["save", "load"])
    parser.add_argument("file", help="file name in setup/lighthouse_configs (or a full path)")
    parser.add_argument("--uri", default=URI, help=f"radio URI (default {URI})")
    args = parser.parse_args()

    path = Path(args.file)
    if not path.is_absolute() and path.parent == Path("."):
        path = CONFIG_DIR / path
    path.parent.mkdir(parents=True, exist_ok=True)

    cflib.crtp.init_drivers()
    print(f"connecting to {args.uri} ...")
    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache=str(CACHE_DIR))) as scf:
        scf.wait_for_params()
        if args.action == "save":
            save(scf.cf, path)
        else:
            load(scf.cf, path)


if __name__ == "__main__":
    main()
