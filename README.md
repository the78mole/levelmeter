The arduino based (liquid) levelmeter
=====================================

Ultrasound distance module based liquid levelmeter with arduino micro pro

Description
===========

This project aims at a simple distance measurement unit that can be triggered by serial bus activity. It uses
the arduino micro pro and a HC-SR04 unit, sold by many suppliers e.g. from aliexpress for only a few dollar.

Given a trigger on serial bus, it measures the travelling time of the sound wave, calculates the distance in
millimeters from it and sends it back on the serial bus. This can then easily be further processed e.g. by a 
RaspberryPi.

HowTo build the project
=======================

To build the project yourself, simply download Eclipse for C developers from http://www.eclipse.org and install
the Eclipse AVR plugin from http://avr-eclipse.sourceforge.net. Then fetch the git repo and import the project into
your Eclipse workspace. The programmer used for the micro pro is the AVR109 compatible programmer over USB (/dev/ttyACM0).

For programming, you need to reset the device (pull reset pin to ground) and start programming from Eclipse within a few seconds.
