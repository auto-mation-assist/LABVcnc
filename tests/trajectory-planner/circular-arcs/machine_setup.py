#!/usr/bin/env python
'''Copied from m61-test'''

import labvcnc
from labvcnc_control import  LabvcncControl
import hal

from time import sleep
import sys
import os


"""Run the test"""
#Hack to make this wait while LCNC loads
sleep(3)

h = hal.component("python-ui")
h.ready() # mark the component as 'ready'

#
# connect to LabvCNC
#

e = LabvcncControl(1)
e.g_raise_except = False
e.set_mode(labvcnc.MODE_MANUAL)
e.set_state(labvcnc.STATE_ESTOP_RESET)
e.set_state(labvcnc.STATE_ON)
e.do_home(-1)
sleep(1)
e.set_mode(labvcnc.MODE_AUTO)
if len(sys.argv)>1:
    e.open_program(sys.argv[1])
    e.run_full_program()
    sleep(2)
    e.wait_on_program()

else:
    print "No G code specified, setup complete"


