#!/bin/bash -e

do_test() {
    INI=$1
    labvcnc -r $INI | grep -i m6
}

echo "**********  Testing python remaps"
do_test test-py.ini
echo
echo "**********  Testing ngc remaps"
do_test test-ngc.ini

