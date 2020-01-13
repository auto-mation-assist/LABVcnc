#!/bin/bash -e

for i in `seq 20`; do
    labvcnc -r test.ini
    if grep -q '^[^+].*Segmentation fault' stderr; then
	exit 1
    fi
done
