/********************************************************************
* Description: lbvglb.h
*   Declarations for globals found in lbvglb.c
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
#ifndef LBVGLB_H
#define LBVGLB_H

#include "config.h"             /* LINELEN */
#include "math.h"		/* M_PI */
#include "lbvmotcfg.h"          /* LBVMOT_MAX_DIO */
#include "debugflags.h"

#ifdef __cplusplus
extern "C" {
#endif

    extern char lbv_inifile[LINELEN];

    extern char lbv_nmlfile[LINELEN];

#define DEFAULT_RS274NGC_STARTUP_CODE ""
    extern char rs274ngc_startup_code[LINELEN];

/* debug bitflags */
/* Note: these may be hard-code referenced by the GUI (e.g., lbvdebug.tcl).
   If you change the assignments here, make sure and reflect that in
   the GUI scripts that use these. Unfortunately there's no easy way to
   get these into Tk automatically */

    // there's also an lbv_Debug function in lbv/usr_intf/lbvsh.cc
    extern int lbv_debug;

    // LBV_DEBUG_* flag definitions moved to debugflags.h

    extern double lbv_task_cycle_time;	

    extern double lbv_io_cycle_time;

    extern int lbv_task_interp_max_len;

    extern char tool_table_file[LINELEN];

    extern struct LbvPose tool_change_position;
    extern unsigned char have_tool_change_position;


/*just used to keep track of unneccessary debug printing. */
    extern int taskplanopen;

    extern int lbvGetArgs(int argc, char *argv[]);

typedef struct JointConfig_t {
    int Inited;
    unsigned char Type;   // non-zero means joint called init
    double Units;
    double MaxVel;
    double MaxAccel;
    double MinLimit;
    double MaxLimit;
} JointConfig_t;

typedef struct AxisConfig_t {
    int Inited;
    unsigned char Type;
    double MaxVel;
    double MaxAccel;
    double Home;
    double MinLimit;
    double MaxLimit;
} AxisConfig_t;

typedef struct TrajConfig_t {
    int Inited;	// non-zero means traj called init
    int Joints;
    int Spindles;
    double MaxAccel;
    double MaxVel;
    int DeprecatedAxes;
    int AxisMask;
    double LinearUnits;
    double AngularUnits;
    int MotionId;
} TrajConfig_t;

#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif
#endif				/* LBVGLB_H */
