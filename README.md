# Hitec Optima PPM/SBUS

** UNDER WORK **

**Only attempt this modification if you know what you are doing and understand the risks.**

**Flashing this firmware is not reversible and will void your warranty. I'm not responsible for anything that happens. Use at your own risk!**

The lack of PPM output on Hitec Optima receivers is disappointing. To fill the void I have written firmware that will work with the regular Optima series (Optima Lite not tested) with a combined PPM or SBUS output. 

Optima-PPM-SBUS poject is created taking as a base the good work of https://github.com/zero3nna/Optima-PPM, but adding/changing the next features:
 - FailSafe feature added. Official Hitech FailSafe Enable and Disable with button is working.
 - Digital communication oputput can be selected between PPM or SBUS.
 - Compilation migrated to Arduino IDE + [MiniCore](https://github.com/MCUdude/MiniCore)

There are three variations of the firmware:

1. MIXED MODE SERIAL PPM
  * Regular PWM is output on the lower channels of the receiver (1 to N-1) and combined PPM on the highest channel (N).
2. MIXED MODE SERIAL SBUS
  * Regular PWM is output on the lower channels of the receiver (1 to N-1) and SBUS on the highest channel (N).
2. MIXED MODE SELECTABLE PPM/SBUS
  * Regular PWM is output on the lower channels of the receiver (1 to N-2) 
  * Selected SBUS or PPM on the highest channel (N).
  * Channel N-1 to select between PPM or SBUS, connect the signal pin to ground (with a bind plug) to select PPM, otherwise SBUS is selected.
  
  
All builds are available as .hex-file: TODO

If you want to build the firmware yourself, you will need to set up a proper build environment in Adduino:
- TODO


**Make sure you flash the correct microcontroller. You want the slave, not the controller for wireless transfer.** Optima 6 and 7 have the pads in similar locations.

Next step is to solder the corresponding cables to the Optima reciever pins.
OPTIMA 6 ICSP connections:
![Optima 6 Reciever](https://github.com/ciafe/Optima-PPM-SBUS/blob/master/images/Optima6_ICSP.jpeg)

OPTIMA 7 ICSP connections:
![Optima 7 Receiver](https://github.com/ciafe/Optima-PPM-SBUS/blob/master/images/Optima7_ICSP.JPG)

I powered the reciever directly over the SPC-Pins with a lipo battery.


Thats it! We can now use our reciever in PPM-Mode.




