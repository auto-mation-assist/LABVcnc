#!/usr/bin/env python

import labvcnc
import labvcnc_util
import hal

import time
import sys
import os
import math

# this is how long we wait for labvcnc to do our bidding
timeout = 5.0


# unbuffer stdout
sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', 0)


#
# connect to LabvCNC
#

c = labvcnc.command()
e = labvcnc.error_channel()
s = labvcnc.stat()

l = labvcnc_util.LabvCNC(command=c, status=s, error=e)

c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.mode(labvcnc.MODE_MANUAL)   
c.home(-1)   
l.wait_for_home(joints=[1,0,1,0,0,0,0,0,0])


#
# do some jogs in manual mode
#

l.jog_axis('x', -0.5)
l.jog_axis('x', 0.0)

l.jog_axis('z', -0.5)
l.jog_axis('z', 0.0)


#
# do some MDI commands
#

c.mode(labvcnc.MODE_MDI)

gcode = 'g0 x2 z2'
print "running gcode:", gcode
c.mdi(gcode)
c.wait_complete()
l.wait_for_axis_to_stop_at('x', 2);
l.wait_for_axis_to_stop_at('z', 2);

gcode = 'g0 x-2'
print "running gcode:", gcode
c.mdi(gcode)
c.wait_complete()
l.wait_for_axis_to_stop_at('x', -2);
l.wait_for_axis_to_stop_at('z', 2);

gcode = 'g0 z-2'
print "running gcode:", gcode
c.mdi(gcode)
c.wait_complete()
l.wait_for_axis_to_stop_at('x', -2);
l.wait_for_axis_to_stop_at('z', -2);

sys.exit(0)

