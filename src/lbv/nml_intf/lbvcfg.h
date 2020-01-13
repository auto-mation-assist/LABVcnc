/********************************************************************
* Description: lbvcfg.h
*   Compile-time defaults for LBV application. Defaults are used to
*   initialize globals in lbvglb.c. Include lbvglb.h to access these
*   globals.
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
#ifndef LBVCFG_H
#define LBVCFG_H

#ifdef __cplusplus
extern "C" {
#endif

/* default name of LBV ini file */
#define DEFAULT_LBV_INIFILE "lbv.ini"

/* default name of LBV NML file */
#define DEFAULT_LBV_NMLFILE LBV2_DEFAULT_NMLFILE

/* cycle time for lbvtask, in seconds */
#define DEFAULT_LBV_TASK_CYCLE_TIME 0.100

/* cycle time for lbvtio, in seconds */
#define DEFAULT_LBV_IO_CYCLE_TIME 0.100

/* default interp len */
#define DEFAULT_LBV_TASK_INTERP_MAX_LEN 1000

/* default name of LBV_TOOL tool table file */
#define DEFAULT_TOOL_TABLE_FILE "tool.tbl"

/* default feed rate, in user units per second */
#define DEFAULT_TRAJ_DEFAULT_VELOCITY 1.0

/* default traverse rate, in user units per second */
#define DEFAULT_TRAJ_MAX_VELOCITY 10.0

/* default joint velocity, in user units per second */
#define DEFAULT_JOINT_MAX_VELOCITY 1.0

/* default joint acceleration, in user units per second per second */
#define DEFAULT_JOINT_MAX_ACCELERATION 1.0

/* default axis velocity, in user units per second */
#define DEFAULT_AXIS_MAX_VELOCITY 1.0

/* default axis acceleration, in user units per second per second */
#define DEFAULT_AXIS_MAX_ACCELERATION 1.0

#ifdef __cplusplus
}				/* matches extern "C" at top */
#endif
#endif
