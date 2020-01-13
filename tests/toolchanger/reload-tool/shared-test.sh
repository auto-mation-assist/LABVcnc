#!/bin/bash
rm -f sim.var
rm -f simpockets.tbl
cp ../simpockets.tbl.save simpockets.tbl
labvcnc -r test.ini
