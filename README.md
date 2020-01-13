# LabvCNC

Sunday, January 12,2020 - Johannes P. Fassotte - Auto-Mation-Assist

Base-1 "name changes only" for use in the start of building
LabvCNC (linuxcnc 2.9.0-pre0 (01/11/2020) used)

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
memory use definition. Due to the large number of changes the
potential for some errors could exist.

To run this version use the name labvcnc after using the normal
compiling procedures such as the below.

cd src
./autogen.sh
./configure --with-realtime=uspace --enable-non-distributable=yes
make clean
make                                                       #
sudo make setuid
cd ..
. ./scripts/rip-environment
labvcnc

--------------------------------------------------------------------

LabvCNC controls CNC machines. It can drive milling machines,
lathes, 3d printers, laser cutters, plasma cutters, robot arms,
hexapods, and more.

# DISCLAIMER

**THE AUTHORS OF THIS SOFTWARE ACCEPT ABSOLUTELY NO LIABILITY FOR ANY
HARM OR LOSS RESULTING FROM ITS USE.**

**IT IS _EXTREMELY_ UNWISE TO RELY ON SOFTWARE ALONE FOR SAFETY.**

**Any machinery capable of harming persons must have provisions for
completely removing power from all motors, etc, before persons enter
any danger area.**

**All machinery must be designed to comply with local and national
safety codes, and the authors of this software can not, and do not,
take any responsibility for such compliance.**

This software is released under the GPLv2, with some parts under the LGPL.
See the file COPYING for more details.

# The Build Process

Refer to the file 'docs/src/code/building-labvcnc.txt' for information
about building and running the software.
