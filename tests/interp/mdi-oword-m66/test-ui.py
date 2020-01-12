#!/usr/bin/env python

import labvcnc
import labvcnc_util
import hal

import time
import sys
import os


# this is how long we wait for labvcnc to do our bidding
timeout = 1.0


#
# set up pins
# shell out to halcmd to net our pins to where they need to go
#

h = hal.component("python-ui")
h.ready()


#
# connect to LabvCNC
#

c = labvcnc.command()
s = labvcnc.stat()
e = labvcnc.error_channel()

l = labvcnc_util.LabvCNC(command=c, status=s, error=e)

c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.home(0)
c.home(1)
c.home(2)

l.wait_for_home([1, 1, 1, 0, 0, 0, 0, 0, 0])

c.mode(labvcnc.MODE_MDI)

c.mdi("O<obug> call [0]")
c.wait_complete()
s.poll()
if s.position[0] != 0:
    print "ended at wrong location (did O-call terminate with error?)"
    sys.exit(1)

c.mdi("O<obug> call [1]")
c.wait_complete()
s.poll()
if abs(s.position[0] - 1) > 1e-5:
    print "ended at wrong location (did O-call terminate with error?)"
    print s.position
    sys.exit(1)
print "done! it all worked"

# if we get here it all worked!
sys.exit(0)

