#/bin/bash
cd ../../../src
#Ugly way to force rebuild of kinematics, which assumes that tp_debug isn't
#used anywhere else...
touch lbv/tp/t[cp]*.[ch]
make EXTRA_DEBUG=''
