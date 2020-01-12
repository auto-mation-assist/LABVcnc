#!/usr/bin/env python

import labvcnc, hal
import sys

# Initialization
c = labvcnc.command()
s = labvcnc.stat()
c.state(labvcnc.STATE_ESTOP_RESET)
c.state(labvcnc.STATE_ON)
c.mode(labvcnc.MODE_MDI)

c.mdi('(print,pre 1)')
c.mdi('(print,pre 2)')
c.mdi('M400')
c.mdi('(print,post 1)')
c.mdi('(print,post 2)')
c.mdi('(print,post 3)')
c.mdi('(print,post 4)')
c.mdi('(print,post 5)')
c.mdi('(print,post 6)')

# Shutdown
c.wait_complete()
sys.exit(0)

