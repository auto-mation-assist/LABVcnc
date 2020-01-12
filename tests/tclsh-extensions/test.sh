#!/bin/bash
# (runtests needs to run without a display, but LABVCNC_LBVSH is always a
# wish-like program. Assume we can get the related non-wish interpreter in the
# obvious way)
realtime start
${LABVCNC_LBVSH/wish/tclsh} test.tcl; exitval=$?
realtime stop
exit $exitval
