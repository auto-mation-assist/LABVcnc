/********************************************************************
* Description: lbvsvr.cc
*   Network server for LBV NML
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <stdio.h>		// sscanf()
#include <math.h>		// fabs()
#include <stdlib.h>		// exit()
#include <string.h>		// strncpy()
#include <unistd.h>             // _exit()
#include <signal.h>

#include "rcs.hh"		// LBV NML
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"		// LBV NML
#include "lbvglb.h"		// lbvGetArgs(), LBV_NMLFILE
#include "inifile.hh"
#include "rcs_print.hh"
#include "nml_oi.hh"
#include "timer.hh"
#include "nml_srv.hh"           // run_nml_servers()

static int tool_channels = 1;

static int iniLoad(const char *filename)
{
    IniFile inifile;
    const char *inistring;

    // open it
    if (inifile.Open(filename) == false) {
	return -1;
    }

    if (NULL != (inistring = inifile.Find("DEBUG", "LBV"))) {
	// copy to global
	if (1 != sscanf(inistring, "%x", &lbv_debug)) {
	    lbv_debug = 0;
	}
    } else {
	// not found, use default
	lbv_debug = 0;
    }
    if (lbv_debug & LBV_DEBUG_RCS) {
	set_rcs_print_flag(PRINT_EVERYTHING);
	max_rcs_errors_to_print = -1;
    }

    if (NULL != (inistring = inifile.Find("NML_FILE", "LBV"))) {
	// copy to global
	strcpy(lbv_nmlfile, inistring);
    } else {
	// not found, use default
    }
    inifile.Find(&tool_channels,"TOOL_CHANNELS","LBV");
    // close it
    inifile.Close();

    return 0;
}

// based on code from
// http://www.microhowto.info/howto/cause_a_process_to_become_a_daemon_in_c.html
static void daemonize()
{
    pid_t pid = fork();
    if (pid < 0) {
        perror("daemonize: fork()");
    } else if (pid) {
        _exit(0);
    }

    if(setsid() < 0)
        perror("daemonize: setsid()");

    // otherwise the parent may deliver a SIGHUP to this process when it
    // terminates
    signal(SIGHUP,SIG_IGN);

    pid=fork();
    if (pid < 0) {
        perror("daemonize: fork() 2");
    } else if (pid) {
        _exit(0);
    }
}

static RCS_CMD_CHANNEL *lbvCommandChannel = NULL;
static RCS_STAT_CHANNEL *lbvStatusChannel = NULL;
static NML *lbvErrorChannel = NULL;
static RCS_CMD_CHANNEL *toolCommandChannel = NULL;
static RCS_STAT_CHANNEL *toolStatusChannel = NULL;

int main(int argc, char *argv[])
{
    double start_time;

    // process command line args
    if (0 != lbvGetArgs(argc, argv)) {
	rcs_print_error("Error in argument list\n");
	exit(1);
    }
    // get configuration information
    iniLoad(lbv_inifile);

    set_rcs_print_destination(RCS_PRINT_TO_NULL);

    rcs_print("after iniLoad()\n");


    start_time = etime();

    while (fabs(etime() - start_time) < 10.0 &&
	   (lbvCommandChannel == NULL || lbvStatusChannel == NULL
	    || (tool_channels && (toolCommandChannel == NULL || toolStatusChannel == NULL))
	    || lbvErrorChannel == NULL)
	) {
	if (NULL == lbvCommandChannel) {
	    rcs_print("lbvCommandChannel==NULL, attempt to create\n");
	    lbvCommandChannel =
		new RCS_CMD_CHANNEL(lbvFormat, "lbvCommand", "lbvsvr",
				    lbv_nmlfile);
	}
	if (NULL == lbvStatusChannel) {
	    rcs_print("lbvStatusChannel==NULL, attempt to create\n");
	    lbvStatusChannel =
		new RCS_STAT_CHANNEL(lbvFormat, "lbvStatus", "lbvsvr",
				     lbv_nmlfile);
	}
	if (NULL == lbvErrorChannel) {
	    lbvErrorChannel =
		new NML(nmlErrorFormat, "lbvError", "lbvsvr", lbv_nmlfile);
	}
	if (tool_channels) {
	    if (NULL == toolCommandChannel) {
		toolCommandChannel =
		    new RCS_CMD_CHANNEL(lbvFormat, "toolCmd", "lbvsvr",
					lbv_nmlfile);
	    }
	    if (NULL == toolStatusChannel) {
		toolStatusChannel =
		    new RCS_STAT_CHANNEL(lbvFormat, "toolSts", "lbvsvr",
					 lbv_nmlfile);
	    }
	}

	if (!lbvCommandChannel->valid()) {
	    delete lbvCommandChannel;
	    lbvCommandChannel = NULL;
	}
	if (!lbvStatusChannel->valid()) {
	    delete lbvStatusChannel;
	    lbvStatusChannel = NULL;
	}
	if (!lbvErrorChannel->valid()) {
	    delete lbvErrorChannel;
	    lbvErrorChannel = NULL;
	}
	if (tool_channels) {
	    if (!toolCommandChannel->valid()) {
		delete toolCommandChannel;
		toolCommandChannel = NULL;
	    }
	    if (!toolStatusChannel->valid()) {
		delete toolStatusChannel;
		toolStatusChannel = NULL;
	    }
	}
	esleep(0.200);
    }

    set_rcs_print_destination(RCS_PRINT_TO_STDERR);

    if (NULL == lbvCommandChannel) {
	lbvCommandChannel =
	    new RCS_CMD_CHANNEL(lbvFormat, "lbvCommand", "lbvsvr",
				lbv_nmlfile);
    }
    if (NULL == lbvStatusChannel) {
	lbvStatusChannel =
	    new RCS_STAT_CHANNEL(lbvFormat, "lbvStatus", "lbvsvr",
				 lbv_nmlfile);
    }
    if (NULL == lbvErrorChannel) {
	lbvErrorChannel =
	    new NML(nmlErrorFormat, "lbvError", "lbvsvr", lbv_nmlfile);
    }
    if (tool_channels) {
	if (NULL == toolCommandChannel) {
	    toolCommandChannel =
		new RCS_CMD_CHANNEL(lbvFormat, "toolCmd", "lbvsvr",
				    lbv_nmlfile);
	}
	if (NULL == toolStatusChannel) {
	    toolStatusChannel =
		new RCS_STAT_CHANNEL(lbvFormat, "toolSts", "lbvsvr",
				     lbv_nmlfile);
	}
    }
    daemonize();
    run_nml_servers();

    return 0;
}

// vim:sw=4:sts=4:et
