# setup

getting a drone from out of the box to flying with the scripts in this repo.
do these in order the first time, after that just `check_drone.py` before flying.

```
README.md             this
scan.py               find drones the radio can see
check_drone.py        connect + check decks, battery, lighthouse, position (no flying)
lighthouse_config.py  save / load the base station config to a yaml file
lighthouse_configs/   saved configs (the 2024_* ones are from the old lab setup)
```

## what you need

- crazyflie 2.1 (or 2.1+ / brushless) with the lighthouse deck on top
- crazyradio (pa or 2.0)
- 2+ lighthouse v2 base stations, ~50 cm above the flight area, no mirrors/sunlight

## 1. install

from the repo root:

```
python -m venv .venv
.venv\Scripts\activate
pip install -r requirements.txt
pip install cfclient
```

start the client with `cfclient`.

## 2. radio driver (windows)

the crazyradio needs the libusb driver or nothing can see it.

1. plug in the crazyradio
2. get zadig (zadig.akeo.ie), select the crazyradio in the list
   (options > list all devices if its not there)
3. pick **libusb-win32**, click install
4. device manager should now show it under libusb devices

if it connects only sometimes, redo it with **libusbK** instead.

## 3. first connect

1. turn the drone on, open cfclient
2. leave the address as `0xE7E7E7E7E7` (factory default), hit **scan**
3. pick the uri in the dropdown, hit **connect**

or `python setup/scan.py` without cfclient.

## 4. firmware

do this with the lighthouse deck mounted, it updates the deck too.

1. connect > bootloader
2. connect to the drone in that window
3. from release > pick the latest for your platform (cf2.1 etc)
4. program, wait for it to finish and restart

## 5. radio address / channel

every drone needs its own uri, otherwise two drones answer the same radio.

1. connect to the drone
2. connect > configure 2.x
3. set channel (0-125), datarate (2M) and address
4. write, then power cycle the drone and connect with the new uri

for several drones on one crazyradio, keep the same channel and change only
the last byte of the address (E7E7E7E701, E7E7E7E702, ...). the radio doesnt
have to hop channels that way. write the label on the drone.

| drone | uri | notes |
|---|---|---|
| | radio://0/80/2M/E7E7E7E7E9 | 2024 lab default |
| | | |

the uri then goes in `URI = ...` at the top of whatever script you run,
or `--uri radio://...` on the command line.

## 6. lighthouse

in cfclient: view > tabs > lighthouse positioning

1. **set bs channel**: give each base station its own channel (1, 2, ...).
   one at a time, wait for "success!"
2. connect to a drone, **switch bs version** to v2 if its not
3. hold the drone in the space for ~20 s so it gets the calibration
4. **start set up** and follow it: origin, +x at 1 m, a few xy floor samples,
   then xyz samples around the flight volume
5. it uploads to the drone when done (status says uploaded)

check the grid: receiving green for every base station, calibration green,
geometry green. errors over ~10 mm in sample details means redo some samples.

### saving it

the geometry lives on the drone. save it so other drones (or this one after a
reflash) dont need the whole setup again:

```
python setup/lighthouse_config.py save lab_2026_10.yaml
python setup/lighthouse_config.py load lab_2026_10.yaml --uri radio://0/80/2M/E7E7E7E701
```

cfclient's **export configuration** / **import configuration** buttons do the
same thing. if a base station gets bumped, redo step 4 and save again.

the `2024_*` configs only work if the base stations are still exactly where
they were in july 2024. probably not, so redo the setup and save a new one.

## 7. check before flying

```
python setup/check_drone.py --uri radio://0/80/2M/E7E7E7E7E9
```

it doesnt take off. look for: lighthouse deck found, battery > 3.8 V,
all base stations received, position steady (not drifting).

then `test/01_keyboard_flight.py`.

## problems

- **scan finds nothing**: radio driver (step 2), drone on, right address in the scan box
- **connects then drops**: battery, or two drones on the same uri
- **position jumps / drifts**: base stations blocked or bumped, redo geometry
- **calibration orange**: base station changed since the geometry was made, redo step 4
- **wont take off**: newer firmware needs arming, the scripts do this. in cfclient use the arm button
- **pip install cfclient fails**: try python 3.12 in its own venv
