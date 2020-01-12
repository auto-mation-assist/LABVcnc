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


def wait_for_mdi_queue(queue_len, timeout=10):
    start_time = time.time()
    while (time.time() - start_time) < timeout:
        s.poll()
        if s.queued_mdi_commands == queue_len:
            return
        time.sleep(0.1)
    print "queued_mdi_commands at %d after %.3f seconds" % (s.queued_mdi_commands, timeout)
    sys.exit(1)


c = labvcnc.command()
s = labvcnc.stat()
e = labvcnc.error_channel()


h = hal.component("test-ui")
h.newpin("digital-poker", hal.HAL_BIT, hal.HAL_OUT)
h['digital-poker'] = False
h.ready()

hal.new_sig('poke', hal.HAL_BIT)
hal.connect('motion.digital-in-00', 'poke')
hal.connect('test-ui.digital-poker', 'poke')


# Wait for LabvCNC to initialize itself so the Status buffer stabilizes.
wait_for_labvcnc_startup(s)

c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.home(-1)
c.wait_complete()

c.mode(labvcnc.MODE_MDI)

# At startup there's nothing in the queue.
s.poll()
assert(s.queued_mdi_commands == 0)

# Block Motion from draining the queue, by asking it to wait for us to
# poke a synchronized digital input.  Wait up to 30 seconds for digital
# input 0 to go High.
c.mdi('m66 p0 l3 q30')

s.poll()
assert(s.queued_mdi_commands == 0)

# Put an MDI command on Task's MDI queue.
c.mdi('g4 p0')

s.poll()
assert(s.queued_mdi_commands == 1)

# Add another MDI command that we can control.
# Wait up to 30 seconds for digital input 0 to go Low.
c.mdi('m66 p0 l4 q30')

s.poll()
assert(s.queued_mdi_commands == 2)

c.mdi('g4 p0')

s.poll()
assert(s.queued_mdi_commands == 3)

h['digital-poker'] = True
wait_for_mdi_queue(queue_len=1, timeout=10)

h['digital-poker'] = False
wait_for_mdi_queue(queue_len=0, timeout=10)

sys.exit(0)

