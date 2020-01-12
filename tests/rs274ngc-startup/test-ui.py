#!/usr/bin/env python

import labvcnc
import labvcnc_util
import hal

import math
import time
import sys
import subprocess
import os
import signal
import glob
import re

retval = 0


c = labvcnc.command()
s = labvcnc.stat()
e = labvcnc.error_channel()
l = labvcnc_util.LabvCNC(command=c, status=s, error=e)

l.wait_for_labvcnc_startup()

s.poll()

if s.g5x_index != 1:
    print "Expected g5x_index=1 (startup in G54), got %d instead" % s.g5x_index
    retval = 1

if math.fabs(s.tool_offset[2] - 0.1234) > 0.000001:
    print "Expected tool offset of 0.1234 via startup gcode not detected"
    print "Got %f instead." % s.tool_offset[2]
    retval = 1

sys.exit(retval)

