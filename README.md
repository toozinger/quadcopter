# Quadcopter

A super awesome home built quadcopter

# Building and Uploading

Use `quatcopter.py` to build and upload easier. The script has four commands, `clean, build, upload and serial`. To use them, you simply add the first letter of the command as the first arg.

    Example:
    ./quadcopter bus
    This will build code, upload it then start up the serial interface

# Goals

Make flying flat =0.5, not zero.

Make motor power from -1 to 1, then scale to 20 to 70 pwm the motors.

First, only try with P PID, then add I and D simultaneously, later.
