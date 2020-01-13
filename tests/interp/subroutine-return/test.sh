#!/bin/bash

cp -f orig.ngc test.ngc
cp -f subs/orig-sub.ngc subs/sub.ngc

labvcnc -r interp.ini
exit $?

