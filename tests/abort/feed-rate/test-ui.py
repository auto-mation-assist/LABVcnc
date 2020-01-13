#!/usr/bin/env python

import labvcnc
import hal

import math
import time
import sys
import subprocess
import os
import signal
import glob
import re


def wait_for_labvcnc_startup(status, timeout=10.0):

    """Poll the Status buffer waiting for it to look initialized,
    rather than just allocated (all-zero).  Returns on success, throws
    RuntimeError on failure."""

    start_time = time.time()
    while time.time() - start_time < timeout:
        status.poll()
        if (status.angular_units == 0.0) \
            or (status.axes == 0) \
            or (status.axis_mask == 0) \
            or (status.cycle_time == 0.0) \
            or (status.exec_state != labvcnc.EXEC_DONE) \
            or (status.interp_state != labvcnc.INTERP_IDLE) \
            or (status.inpos == False) \
            or (status.linear_units == 0.0) \
            or (status.max_acceleration == 0.0) \
            or (status.max_velocity == 0.0) \
            or (status.program_units == 0.0) \
            or (status.rapidrate == 0.0) \
            or (status.state != labvcnc.STATE_ESTOP) \
            or (status.task_state != labvcnc.STATE_ESTOP):
            time.sleep(0.1)
        else:
            # looks good
            return

    # timeout, throw an exception
    raise RuntimeError


c = labvcnc.command()
s = labvcnc.stat()
e = labvcnc.error_channel()

# Wait for LabvCNC to initialize itself so the Status buffer stabilizes.
wait_for_labvcnc_startup(s)

c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.home(-1)
c.wait_complete()

c.mode(labvcnc.MODE_MDI)

feed_rate = 234.5
c.mdi('g94 ; normal "units per minute" feed mode')
c.mdi('f%.1f' % feed_rate)
c.wait_complete()
c.abort()
c.wait_complete()
s.poll()
print "feed rate:", s.settings[1]
assert(math.fabs(s.settings[1] - feed_rate) < 0.0000001)

feed_rate = 345.6
c.mdi('g95 ; "units per revolution" feed mode')
c.mdi('f%.1f' % feed_rate)
c.wait_complete()
c.abort()
c.wait_complete()
s.poll()
print "feed rate:", s.settings[1]
assert(math.fabs(s.settings[1] - feed_rate) < 0.0000001)

sys.exit(0)

