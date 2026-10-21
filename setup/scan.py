"""find drones the crazyradio can see

python setup/scan.py                          # factory address E7E7E7E7E7
python setup/scan.py E7E7E7E7E9 E7E7E7E701    # check other addresses
"""

import argparse

import cflib.crtp

DEFAULT_ADDRESS = "E7E7E7E7E7"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("addresses", nargs="*", default=[DEFAULT_ADDRESS], help="hex radio addresses")
    args = parser.parse_args()

    cflib.crtp.init_drivers()
    found = []
    for address in args.addresses:
        print(f"scanning {address} ...")
        for uri, _ in cflib.crtp.scan_interfaces(int(address, 16)):
            if uri.startswith("radio://"):
                print(f"  {uri}")
                found.append(uri)

    if not found:
        print("nothing found. is the drone on, is the radio driver installed (setup/README.md step 2), right address?")


if __name__ == "__main__":
    main()
