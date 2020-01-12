/********************************************************************
* Description: usrmotintf.h
*   Decls for interface functions (init, exit, read, write) for user
*   processes which communicate with the real-time motion controller
*   in lbvmot.c
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/
#ifndef USRMOTINTF_H
#define USRMOTINTF_H

struct lbvmot_status_t;
struct lbvmot_command_t;
struct lbvmot_config_t;
struct lbvmot_debug_t;
struct lbvmot_error_t;

#ifdef __cplusplus
extern "C" {
#endif

/* usrmotIniLoad() loads params (SHMEM_KEY) from
   named ini file */
    extern int usrmotIniLoad(const char *file);

/* usrmotReadLbvmotStatus() gets the status info out of
   the lbvmot controller and puts it in arg */
    extern int usrmotReadLbvmotStatus(lbvmot_status_t * s);

/* usrmotReadLbvmotConfig() gets the config info out of
   the lbvmot controller and puts it in arg */
    extern int usrmotReadLbvmotConfig(lbvmot_config_t * s);

/* usrmotReadLbvmotDebug() gets the debug info out of
   the lbvmot controller and puts it in arg */
    extern int usrmotReadLbvmotDebug(lbvmot_debug_t * s);

/* usrmotReadLbvmotError() gets the earliest queued error string out of
   the lbvmot controller and puts it in arg */
    extern int usrmotReadLbvmotError(char *e);

/* usrmotPrintLbvmotStatus() prints the status in s, using which
   arg to select sub-prints */
    extern void usrmotPrintLbvmotStatus(lbvmot_status_t *s, int which);

/* usrmotPrintLbvmotConfig() prints the config in s, using which
   arg to select sub-prints */
    extern void usrmotPrintLbvmotConfig(lbvmot_config_t s, int which);

/* usrmotPrintLbvmotDebug() prints the debug in s, using which
   arg to select sub-prints */
    extern void usrmotPrintLbvmotDebug(lbvmot_debug_t *s, int which);

/* values returned by usrmotWriteLbvmotCommand; negative values
   are all errors */
#define LBVMOT_COMM_OK 0	/* went through and honored */
#define LBVMOT_COMM_ERROR_CONNECT -1	/* can't even connect */
#define LBVMOT_COMM_ERROR_TIMEOUT -2	/* connected, but send timeout */
#define LBVMOT_COMM_ERROR_COMMAND -3	/* sent, but can't run command now */
#define LBVMOT_COMM_SPLIT_READ_TIMEOUT -4	/* can't read without split */
#define LBVMOT_COMM_INVALID_MOTION_ID -5 /* do not queue a motion id MOTION_INVALID_ID */

/* usrmotWriteLbvmotCommand() writes the command to the lbvmot process.
   Return values are as per the #defines above */
    extern int usrmotWriteLbvmotCommand(lbvmot_command_t * c);

/* usrmotInit() initializes communication with the lbvmot process */
    extern int usrmotInit(const char *name);

/* usrmotExit() terminates communication with the lbvmot process */
    extern int usrmotExit(void);

/* usrmotLoadComp() loads the compensation data in file into the joint */
    extern int usrmotLoadComp(int joint, const char *file, int type);

/* usrmotPrintComp() prints the joint compensation data for the specified joint */
    extern int usrmotPrintComp(int joint);

#ifdef __cplusplus
}
#endif
#endif				/* USRMOTINTF_H */
