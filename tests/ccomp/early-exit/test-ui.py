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

#time.sleep(9999)

c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.home(-1)
c.wait_complete()

c.mode(labvcnc.MODE_MDI)
c.mdi('g0 x0.1 y0.2 z0.3')
c.mdi('g41.1 d0.1')
c.mdi('g40')
c.mdi('g0 x0.9  ; surprise motion on Y and Z, to 0!!')
c.wait_complete()

s.poll()
print "position:", s.position
assert(math.fabs(s.position[0] - 0.9) < 0.0000001)
assert(math.fabs(s.position[1] - 0.2) < 0.0000001)
assert(math.fabs(s.position[2] - 0.3) < 0.0000001)

sys.exit(0)

