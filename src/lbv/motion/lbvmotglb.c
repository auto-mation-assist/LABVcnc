/********************************************************************
* Description: lbvmotglb.c
*   Compile-time configuration parameters
*
*   Set the values in lbvmotcfg.h; these vars will be set to those values
*   and lbvmot.c can reference the variables with their defaults. This file
*   exists to avoid having to recompile lbvmot.c every time a default is
*   changed.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include "lbvmotglb.h"		/* these decls */
#include "lbvmotcfg.h"		/* initial values */

char LBVMOT_INIFILE[LBVMOT_INIFILE_LEN] = DEFAULT_LBVMOT_INIFILE;


unsigned int SHMEM_KEY = DEFAULT_SHMEM_KEY;

double LBVMOT_COMM_TIMEOUT = DEFAULT_LBVMOT_COMM_TIMEOUT;

double VELOCITY = DEFAULT_VELOCITY;
double ACCELERATION = DEFAULT_ACCELERATION;

double MAX_LIMIT = DEFAULT_MAX_LIMIT;
double MIN_LIMIT = DEFAULT_MIN_LIMIT;

int TC_QUEUE_SIZE = DEFAULT_TC_QUEUE_SIZE;

double MAX_FERROR = DEFAULT_MAX_FERROR;
