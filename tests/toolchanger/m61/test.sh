#!/bin/bash

cp -f ../simpockets.tbl.original simpockets.tbl

labvcnc -r m61-test.ini
exit $?

