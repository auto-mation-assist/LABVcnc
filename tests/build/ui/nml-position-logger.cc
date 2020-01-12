// g++ nml-position-logger.cc -I include -L lib -l nml -l labvcnc
#include "lbv.hh"
#include "lbv_nml.hh"
#include <unistd.h>
#include <iostream>
#include <cstdlib>

int main(int argc, char **argv) {
    if(argc < 2) { std::cerr << "Usage: " << argv[0] << " NMLFILE\n"; abort(); }
    const char *nmlfile = argv[1];
    RCS_STAT_CHANNEL *stat = new RCS_STAT_CHANNEL(lbvFormat, "lbvStatus", "xlbv", nmlfile);
    while(1) {
        usleep(100*1000);
        if(!stat->valid()) continue;
        if(stat->peek() != LBV_STAT_TYPE) continue;
        LBV_STAT *lbvStatus = static_cast<LBV_STAT*>(stat->get_address());
        std::cout << lbvStatus->motion.traj.position.tran.x << " "
            << lbvStatus->motion.traj.position.tran.y << " "
            << lbvStatus->motion.traj.position.tran.z << "\n";
    }
    return 0;
}
