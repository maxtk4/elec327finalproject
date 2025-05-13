Deepak Narayan and Max Kuhlman
Spring 2025
ELEC 327 Final Project 'Payload'
Professore Kemere


This file details the code architecture of our final project, a
robotic payload for the SAE Aero Design Advanced 2025 competition.

Architecture:
- Based around a state machine, which handles input from limit switches to determine
which state the payload is in
- uses SPI for communicating with neopixels
	+ The SPI bus is clocked at 8 MHz, such that the neopixels consider each 'byte' to be one bit of data
- uses PWM and the timer module to control a linear servo, with pulses of specific different lengths driving
the output
