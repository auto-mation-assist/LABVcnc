#!/bin/bash

#export PATH=$PATH:$LBV2_HOME/tests/helpers
#source $LBV2_HOME/tests/helpers/test-functions.sh

lbv motion-test.ini &

# let lbv come up
sleep 4 

(
    echo starting to capture data
    halsampler -t -n 20000 >| result.halsamples
    echo finished capturing data
) &

(
    echo hello LBV mt 1.0
    echo set enable LBVTOO

    echo set mode manual
    echo set estop off
    echo set machine on

    echo set home 0
    echo set home 1
    echo set home 2

    # give lbv a second to home
    sleep 1.0

    echo set mode mdi
    echo set mdi g0x1

    # give lbv a half second to move
    sleep 0.5

    echo shutdown
) | nc localhost 5007


# wait for lbv to finish
wait

exit 0

