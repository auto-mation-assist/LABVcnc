#!/bin/bash

rm -f sim.var
cp sim.var.pre sim.var
labvcnc -r test.ini

