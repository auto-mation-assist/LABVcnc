#!/bin/sh
set -x
g++ -I $LBV2_HOME/include \
    nml-position-logger.cc \
    -L $LBV2_HOME/lib -lnml -llabvcnc \
    -o /dev/null
