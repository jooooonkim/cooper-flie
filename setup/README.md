# setup

getting a drone from out of the box to flying with the scripts in this repo, on ubuntu.
everything here is done in cfclient except the last keyboard flight test.

bitcraze getting started: https://www.bitcraze.io/documentation/start/

```
README.md             this
lighthouse_configs/   saved lighthouse system configs (2024_* are from the old lab setup)
```

## what's needed

- crazyflie 2.1 (or 2.1+ / brushless) with the lighthouse deck on top
- crazyradio (pa or 2.0)
- micro usb cable
- 2+ lighthouse v2 base stations, ~50 cm above the flight area, no mirrors/sunlight

https://www.bitcraze.io/products/crazyflie-2-1/

## 1. install

from the repo root:

```
sudo apt install git python3-pip libxcb-xinerama0 libxcb-cursor0
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
pip install cfclient
```

cfclient starts with `cfclient`.

https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/

## 2. radio driver (ubuntu)

linux needs usb permissions for the crazyradio and the crazyflie, otherwise
cfclient cant see them.

```
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER
```

log out and back in, then add the udev rules:

```
cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
```

unplug and replug the crazyradio after.

https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/

## 3. first connect

1. open cfclient
2. connect the battery to the drone
3. connect the drone to the computer with micro usb
4. hit **scan**, look for `usb://0` in the dropdown, then **connect**
5. connect > configure 2.x
6. set the radio channel and address, **write**, then power cycle the drone

the channel and address are what wireless connections and the python scripts
use (`radio://0/<channel>/2M/<address>`). each drone gets its own channel and
address. when flying several at once on the same address, give each one a
different channel. label the drone with its uri.

| drone | uri | notes |
|---|---|---|
| | radio://0/80/2M/E7E7E7E7E9 | 2024 lab default |
| | | |

the uri goes in `URI = ...` at the top of whatever script is run, or
`--uri radio://...` on the command line.

https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/

## 4. firmware

done with the lighthouse deck mounted, it updates the deck too.

1. connect > bootloader
2. connect to the drone in that window
3. from release > pick the latest for the relevant platform (cf2.1 etc)
4. program, wait for it to finish and restart

https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyflie-2-x/

## 5. lighthouse

in cfclient: view > tabs > lighthouse positioning. connect to the drone over
radio first.

1. **set bs channel**: each base station gets its own channel (1, 2, ...).
   one at a time, wait for "success!"
2. **switch bs version** to v2 if its not already
3. hold the drone in the flight area for ~20 s so it picks up the base
   station calibration

### calibration

1. **manage geometry** (**start set up** in newer cfclient)
2. follow what cfclient asks for: origin, +x at 1 m, a few xy floor samples,
   then xyz samples around the flight volume
3. it estimates the geometry and uploads it to the drone (status says uploaded)

the grid in the tab should show receiving, calibration and geometry green for
every base station. errors over ~10 mm in sample details means some samples
need redoing.

### saving the config

the geometry lives directly on the drone, in its own memory. it survives power
off but gets lost on some reflashes, and other drones dont have it.

- **save system config** (**export configuration** in newer cfclient) writes it
  to a yaml file. keep it in `setup/lighthouse_configs/` with the date in the name
- **load system config** (**import configuration**) writes a saved yaml onto a
  drone, so every drone shares the same coordinate frame without redoing the
  calibration

if a base station gets bumped, redo the calibration and save a new config.
the `2024_*` configs only work if the base stations are exactly where they were
in july 2024.

https://www.bitcraze.io/documentation/tutorials/getting-started-with-lighthouse/
https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/lighthouse_tab/

## 6. check before flying

all in cfclient, with the drone connected over radio and sitting in the flight area:

- **battery**: flight control tab, above ~3.8 V
- **base stations**: lighthouse tab, every base station receiving (green)
- **position**: lighthouse tab 3d view, the drone dot stays still and doesnt drift or jump

https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/

## 7. keyboard flight

close cfclient first, only one program can use the crazyradio at a time.

```
python test/01_keyboard_flight.py --uri radio://0/80/2M/E7E7E7E7E9
```

w/a/s/d move, o/p up/down, q land, esc kills the motors.

## problems

- **scan finds nothing**: usb permissions (step 2), drone on, cable is a data cable
- **connects then drops**: low battery, or two drones on the same uri
- **position jumps / drifts**: base stations blocked or bumped, redo the calibration
- **calibration orange**: a base station changed since the geometry was made, redo the calibration
- **wont take off**: newer firmware needs arming. the scripts do this, cfclient has an arm button
- **script cant find the radio**: cfclient still open and holding it
