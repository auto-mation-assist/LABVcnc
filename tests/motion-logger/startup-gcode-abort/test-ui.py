#!/usr/bin/env python

import labvcnc
import sys


#
# connect to LabvCNC
#

c = labvcnc.command()
s = labvcnc.stat()
e = labvcnc.error_channel()


#
# Immediately abort!  Github Issue #49 
#

print "UI abort"
sys.stdout.flush()

c.abort()
c.wait_complete()

print "UI done with abort"
sys.stdout.flush()

