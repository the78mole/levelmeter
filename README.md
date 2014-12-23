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
