#!/bin/bash

cp -f simpockets.tbl.original simpockets.tbl
rm -f sim.var

labvcnc -r g43-test.ini
exit $?

