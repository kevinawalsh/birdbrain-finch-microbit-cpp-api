# birdbrain-finch-microbit-cpp-api

This repository provides a C/C++ API for programming the [BirdBrain Finch 2.0](https://www.birdbraintechnologies.com/products/finch-robot-2-0/) in untethered mode.

This lets you program the Finch in C/C++, producing a hex file that can be
copied over USB to the Finch, which can then run your program fully untethered.

It uses the [CODAL](https://tech.microbit.org/software/runtime/) framework for
the micro:bit v2 that powers the Finch 2.0. So your C++ code can use features of
CODAL, such as cooperative concurrency with lightweight threads (fibers), and
other CODAL and micro:bit features.

For the most part, the Finch API provided here mimics [BirdBrain's Java API](https://learn.birdbraintechnologies.com/slpage/java-installation-for-finch/),
with only a few minor alterations and additions.

## Quick Start

* Clone this repository.
* Edit source/main.cpp, and examine source/Finch.h for API documentation.
* Install the embedded arm toolchain. On MacOS you might try `brew install armmbed/formulae/arm-none-eabi-gcc` or follow the install instructions
  for the [microbit-v2-samples](https://github.com/lancaster-university/microbit-v2-samples).
* Run `python3 build.py` to compile.
* Copy the resulting MICROBIT.hex file to your Finch over USB. For example, plug
  in the USB, then `cp MICROBIT.hex /Volumes/MICROBIT/`
* Be sure to turn on the Finch -- the microbit will boot from USB power without
  turning on the Finch itself, so you need to do that manually. You'll get a sad
  face and warning if you forget.

## Notes

* This is a work in progress, a few methods aren't yet tested or implemented,
  such as the magnetometer/compass and related calibration routine.

* It would be nice to add a song playback feature. And a way to display a few
  old-style emoticons, like...   :-)   :D   :(   :/ 

* The Bluetooth BLE stack is entirely disabled, and this code doesn't use the
  USB uart while running. USB is used for downloading hex files, but is
  otherwise not used.

* Conceivably, USB uart could be enabled for tethered debugging.

* Conceivably, BLE uart could be enabled for robot-to-robot communication.

## Helpful Documentation

The Finch hardware and firmware is only partly documented, and not always
accurately, unfortunately. It helps that much of the software is open-source.
Much of this work was accomplished by examining:

* [Finch SPI Protocol documentation](https://github.com/BirdBrainTechnologies/Protocols/blob/master/MicroBitProtocols.md#finch-spi)

* Source for [Finch's BLE bootloader](https://github.com/Roversa-Robotics/microbit-v2-BirdBrain-BLE)

* Source for [BirdBrain's BlueBird Connector](https://github.com/BirdBrainTechnologies/BlueBirdJava)

* Source for [BirdBrain's Java API](https://learn.birdbraintechnologies.com/slpage/java-installation-for-finch/),

* [micro:bit and CODAL programming docs](https://lancaster-university.github.io/microbit-docs/)

## BirdBrain Finch V2 Languages and Tethered vs Untethered Operation

The [BirdBrain Finch 2.0](https://www.birdbraintechnologies.com/products/finch-robot-2-0/)
official website could be more clear about how these robots are programmed.
There are numerous languages and platforms supported, but they don't work the
same and aren't interchangeable. They fall into two categories:

* **Tethered USB** Program code runs on a laptop/desktop, sending commands over
  USB to control the robot. The robot remains tethered via the USB cable, so has
  a maximum range of just two feet or so, or perhaps a few meters with a long
  USB cable.
* **Tethered Bluetooth** Program code runs on a laptop/desktop, or a
  phone/tablet, sending commands over bluetooth to control the robot. The robot
  must stay within bluetooth range, perhaps a few meters or so. 
* **Untethered** Code is compiled into a hex file, which is uploaded to the
  robot over USB. The robot can then be disconnected and run untethered. The hex
  program is presistent, across robot reboots, until overrwritten.

For all tethered modes, the robot must be running a specific `BBTFirmware.hex`
file, so it can accept bluetooth or USB commands. For desktops/laptops, tethered
modes also require a [BlueBird Connector App](https://learn.birdbraintechnologies.com/install-shortcuts/) daemon.

Languages include:

* **Plain Java**: text-based, command-line, Bluetooth-tethered. This is regular
  java, compiled using javac on the command line, using a simple BirdBrain
  library included with functions for moving, turning, playing sounds, etc.
* **Greenfoot Java**: IDE with graphical simulator, Bluetooth-tethered. Uses a
  bespoke IDE first relased in 2006 but still updated and maintained to some
  degree.
* **C/C++**: text-based, command-line, untethered. Not officially supported by
  BirdBrain, but provided by this repository instead. Compiles directly to hex
  using GCC embdedded ARM toolchain.
* **Microsoft MakeCode**: block-based programming, in-browser, untethered.
  Somewhat like Scratch, but also has a "javascript" view. Compiles to hex in
  the browser, and can either upload to robot via USB directly from browser, or
  you can download the hex file and upload it to the robot over USB as a
  separate step.
* **BirdBlox**: block-based, iOS app for phone or ipad, bluetooth-tethered.
  Block-based, code right in the app on a phone or ipad, almost exactly like Scratch.
  Code runs immediately via bluetooth tether.
* **FinchBlox**: block-based, in-browser, bluetooth-tethered. Similar to
  MakeCode but far more limited programming capabilities, intended for only the
  simplest programs.

## Tethered Bluetooth Architecture

My best guess as to the overall bluetooth architecture, pieced together from
source code and other documents...

    +------ Finch Robot Body -------+----micro:bit v2----+     +---- Laptop Host ----+
    |     SAMD microprocessor       |    ARM Cortex      |     |  Java Program       |
    |     (unknown firmware)        | (BBTFirmware.hex)  |     |   ^                 |
    | with sensors, actuators, etc. |  with BLE radio    |     |   | HTTP to         | 
    +-------------------------------+--------------------+     |   | localhost       |
            ^                           ^   ^                  |   v                 |
            '--- Finch SPI protocol-----'   '--Bluetooth-----> |  BlueBird Connector |
                                                               +---------------------+

