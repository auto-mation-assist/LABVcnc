/********************************************************************
* Description: lbvglb.c
*   Globals initialized to values in lbvcfg.h
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

#include "lbvglb.h"		/* these decls */
#include "lbvcfg.h"		/* their initial values */
#include "lbvpos.h"		/* LbvPose */

char lbv_inifile[LINELEN] = DEFAULT_LBV_INIFILE;

char lbv_nmlfile[LINELEN] = DEFAULT_LBV_NMLFILE;

char rs274ngc_startup_code[LINELEN] =
    DEFAULT_RS274NGC_STARTUP_CODE;

int lbv_debug = 0;		/* initially no debug messages */

double lbv_task_cycle_time = DEFAULT_LBV_TASK_CYCLE_TIME;

double lbv_io_cycle_time = DEFAULT_LBV_IO_CYCLE_TIME;

int lbv_task_interp_max_len = DEFAULT_LBV_TASK_INTERP_MAX_LEN;

char tool_table_file[LINELEN] = DEFAULT_TOOL_TABLE_FILE;

LbvPose tool_change_position;	/* no defaults */
unsigned char have_tool_change_position = 0;	/* default is 'not there' */

int taskplanopen = 0;
