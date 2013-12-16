#!/usr/bin/python2
import sys
import subprocess

# If no args are present, print usage and exit
if len(sys.argv) <= 1:
    print "You must include commands to run"
    print "  c = clean"
    print "  b = build"
    print "  u = upload"
    print "  s = serial"
    print ""
    print "Example: ./quadcopter bus"
    print "This will build code, upload it the start up the serial interface"
    sys.exit()

# Split the first arg into array of charcters
commands = list(sys.argv[1:][0])

# Sort the input chars to they always run in the right order
# If a illegal charcter is found, print message and exit
try:
    commands = sorted(commands, key=lambda word: ["cbus".index(c) for c in word[0]])
except ValueError:
    print "You have included an illegal command."
    print "Your options are 'c, b, u, s'"
    sys.exit()

# Remove any duplicates
commands = list(set(commands))

# Run the commands in order sending output to the terminal
for command in commands:
    if command == "c":
        subprocess.call(["ino", "clean"], stderr=subprocess.STDOUT)
    if command == "b":
        subprocess.call(["ino", "build"], stderr=subprocess.STDOUT)
    if command == "u":
        subprocess.call(["ino", "upload"], stderr=subprocess.STDOUT)
    if command == "s":
        subprocess.call(["ino", "serial", "-b", "115200"], stderr=subprocess.STDOUT)
