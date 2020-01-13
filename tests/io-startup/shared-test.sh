#!/bin/bash

rm -f tool.tbl
cp tool.tbl.original tool.tbl

labvcnc -r test.ini
exit $?

