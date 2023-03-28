This is example of sending CRTP velocity commands from AI-deck to crazyflie over CPX.
# Read me first
NOTE: DO NOT RUN THIS WITH PROPELLERS ON CRAZYFLIE, THE VEHICLE WILL FLY IN RANDOM DIRECTION. REMOVE ALL PROPELLERS BEFORE TESTING.

# Pre-requisits

Please Follow getting started with AI-deck guide from bitcraze https://www.bitcraze.io/documentation/tutorials/getting-started-with-aideck/

# building
```
cd AI_deck_send_vel_send_cpx
docker run --rm -v ${PWD}:/module aideck-with-autotiler tools/build/make-example . image
```
# Flashing

Connect AI-deck with JTAG then run:

```
docker run --rm -v ${PWD}:/module --device /dev/ttyUSB0 --privileged -P bitcraze/aideck tools/build/make-upload-jtag . flash
```
Restart the Crazyflie. After about 20-30 seconds the motors should start spinning.
