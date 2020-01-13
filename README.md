# LABVcnc or LabvCNC 

Having problem uploading the base-1 file that goes with the discription.

Sunday, January 12,2020 - Johannes P. Fassotte - Auto-Mation-Assist

Base-1 "name changes only" for for use in rebuilding
LabvCNC (linuxcnc 2.9.0-pre0 (01/11/2020) used)

To run this version use use the name labvcnc after compiling.

This version of LabvCNC is a modified version of
linuxcnc 2.9.0-pre0 (01/11/2020), and has been modified to use the
name of "labvcnc" and its abreviations in order to allow source
code additions or changes that may be required to allow interfacing
with remote user interfaces generated with Labview. This name change
will aid in preventing potential coding conflicts with ongoing
LinuxCnc work by others.

LabvCNC has as its primary focus the use of remote user interfaces
generated with Labview. This particular version of LinuxCnc provides
the "name" changes that stricly meant to be a flag as to its potential
use with remote interfaces coded with Labview.

Within the code the name or abbreviated name are these:
LABVCNC, LabvCNC, labvCNC, labvcnc, Labvcnc, LBV, Lbv, lbv.
And one of each, LAbvCNC, LabvCnC.

Original names:
LINUXCNC, LinuxCNC, linuxCNC, linuxcnc, Linuxcnc, EMC, Emc, emc.
And one of each, LInuxCNC, LinuxCnC.

Estimated about 75000 total name changes. Note that "emc"
is also part of many other items, as an example "memcpy" a shared
memory use definition. Do to the large number of changes the
potential for errors could exist.
