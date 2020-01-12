#!/usr/bin/env python

import labvcnc
import hal

import time
import sys


#
# connect to LabvCNC
#

c = labvcnc.command()
s = labvcnc.stat()
e = labvcnc.error_channel()


#
# Come out of E-stop, turn the machine on, home, and switch to Auto mode.
#

c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.mode(labvcnc.MODE_AUTO)


#
# run the .ngc test file, starting from the special line
#

c.program_open('test.ngc')
c.auto(labvcnc.AUTO_RUN, 4)
c.wait_complete()

sys.exit(0)
