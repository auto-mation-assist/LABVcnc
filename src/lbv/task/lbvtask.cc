/********************************************************************
* Description: lbvtask.cc
*   Mode and state management for LBV_TASK class
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

#include <stdlib.h>
#include <string.h>		// strncpy()
#include <sys/stat.h>		// struct stat
#include <unistd.h>		// stat()
#include <limits.h>		// PATH_MAX
#include <dlfcn.h>

#include "rcs.hh"		// INIFILE
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"
#include "lbvglb.h"		// LBV_INIFILE
#include "interpl.hh"		// NML_INTERP_LIST, interp_list
#include "canon.hh"		// CANON_VECTOR, GET_PROGRAM_ORIGIN()
#include "rs274ngc_interp.hh"	// the interpreter
#include "interp_return.hh"	// INTERP_FILE_NOT_OPEN
#include "inifile.hh"
#include "rcs_print.hh"
#include "task.hh"		// lbvTaskCommand etc
#include "python_plugin.hh"
#include "taskclass.hh"
#include "motion.h"

#define USER_DEFINED_FUNCTION_MAX_DIRS 5
#define MAX_M_DIRS (USER_DEFINED_FUNCTION_MAX_DIRS+1)
//note:the +1 is for the PROGRAM_PREFIX or default directory==nc_files

/* flag for how we want to interpret traj coord mode, as mdi or auto */
static int mdiOrAuto = LBV_TASK_MODE_AUTO;

InterpBase *pinterp=0;
#define interp (*pinterp)
setup_pointer _is = 0; // helper for gdb hardware watchpoints FIXME


/*
  format string for user-defined programs, e.g., "programs/M1%02d" means
  user-defined programs are in the programs/ directory and are named
  M1XX, where XX is a two-digit string.
*/
static char user_defined_fmt[MAX_M_DIRS][LBV_SYSTEM_CMD_LEN]; // ex: "dirname/M1%02d"

// index to directory for each user defined function:
static int user_defined_function_dirindex[USER_DEFINED_FUNCTION_NUM];

static void user_defined_add_m_code(int num, double arg1, double arg2)
{
    // num      is the m_code number, typically 00-99 corresponding to M100-M199
    char fmt[LBV_SYSTEM_CMD_LEN];
    LBV_SYSTEM_CMD system_cmd;

    //we call FINISH() to flush any linked motions before the M1xx call, 
    //otherwise they would mix badly
    FINISH();
    strcpy(fmt, user_defined_fmt[user_defined_function_dirindex[num]]);
    strcat(fmt, " %f %f");
    snprintf(system_cmd.string, sizeof(system_cmd.string), fmt, num, arg1, arg2);
    interp_list.append(system_cmd);
}

int lbvTaskInit()
{
    char mdir[MAX_M_DIRS][PATH_MAX+1];
    int num,dct,dmax;
    char path[LBV_SYSTEM_CMD_LEN];
    struct stat buf;
    IniFile inifile;
    const char *inistring;

    inifile.Open(lbv_inifile);

    // Identify user_defined_function directories
    if (NULL != (inistring = inifile.Find("PROGRAM_PREFIX", "DISPLAY"))) {
        strncpy(mdir[0],inistring, sizeof(mdir[0]));
        if (mdir[0][sizeof(mdir[0])-1] != '\0') {
            rcs_print("[DISPLAY]PROGRAM_PREFIX too long (max len %zu)\n", sizeof(mdir[0]));
            return -1;
        }
    } else {
        // default dir if no PROGRAM_PREFIX
        strncpy(mdir[0],"nc_files", sizeof(mdir[0]));
        if (mdir[0][sizeof(mdir[0])-1] != '\0') {
            rcs_print("default nc_files too long (max len %zu)\n", sizeof(mdir[0]));
            return -1;
        }
    }
    dmax = 1; //one directory mdir[0],  USER_M_PATH specifies additional dirs

    // user can specify a list of directories for user defined functions
    // with a colon (:) separated list
    if (NULL != (inistring = inifile.Find("USER_M_PATH", "RS274NGC"))) {
        char* nextdir;
        char tmpdirs[PATH_MAX];

        for (dct=1; dct < MAX_M_DIRS; dct++) mdir[dct][0] = 0;

        strncpy(tmpdirs,inistring, sizeof(tmpdirs));
        if (tmpdirs[sizeof(tmpdirs)-1] != '\0') {
            rcs_print("[RS274NGC]USER_M_PATH too long (max len %zu)\n", sizeof(tmpdirs));
            return -1;
        }

        nextdir = strtok(tmpdirs,":");  // first token
        dct = 1;
        while (dct < MAX_M_DIRS) {
            if (nextdir == NULL) break; // no more tokens
            strncpy(mdir[dct],nextdir, sizeof(mdir[dct]));
            if (mdir[dct][sizeof(mdir[dct])-1] != '\0') {
                rcs_print("[RS274NGC]USER_M_PATH component (%s) too long (max len %zu)\n", nextdir, sizeof(mdir[dct]));
                return -1;
            }
            nextdir = strtok(NULL,":");
            dct++;
        }
        dmax=dct;
    }
    inifile.Close();

    /* check for programs named programs/M100 .. programs/M199 and add
       any to the user defined functions list */
    for (num = 0; num < USER_DEFINED_FUNCTION_NUM; num++) {
	for (dct=0; dct < dmax; dct++) {
            char expanddir[LINELEN];
	    if (!mdir[dct][0]) continue;
            if (inifile.TildeExpansion(mdir[dct],expanddir,sizeof(expanddir))) {
		rcs_print("lbvTaskInit: TildeExpansion failed for %s, ignoring\n",
			 mdir[dct]);
            }
	    snprintf(path, sizeof(path), "%s/M1%02d",expanddir,num);
	    if (0 == stat(path, &buf)) {
	        if (buf.st_mode & S_IXUSR) {
		    // set the user_defined_fmt string with dirname
		    // note the %%02d means 2 digits after the M code
		    // and we need two % to get the literal %
		    snprintf(user_defined_fmt[dct], sizeof(user_defined_fmt[0]), 
			     "%s/M1%%02d", expanddir); // update global
		    USER_DEFINED_FUNCTION_ADD(user_defined_add_m_code,num);
		    if (lbv_debug & LBV_DEBUG_CONFIG) {
		        rcs_print("lbvTaskInit: adding user-defined function %s\n",
			     path);
		    }
	            user_defined_function_dirindex[num] = dct;
	            break; // use first occurrence found for num
	        } else {
		    if (lbv_debug & LBV_DEBUG_CONFIG) {
		        rcs_print("lbvTaskInit: user-defined function %s found, but not executable, so ignoring\n",
			     path);
		    }
	        }
	    }
	}
    }

    return 0;
}

int lbvTaskHalt()
{
    return 0;
}

int lbvTaskAbort()
{
    lbvMotionAbort();

    // clear out the pending command
    lbvTaskCommand = 0;
    interp_list.clear();

    // clear out the interpreter state
    lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
    lbvStatus->task.task_paused = 0;
    lbvStatus->task.motionLine = 0;
    lbvStatus->task.readLine = 0;
    lbvStatus->task.command[0] = 0;
    lbvStatus->task.callLevel = 0;

    stepping = 0;
    steppingWait = 0;

    // now queue up command to resynch interpreter
    LBV_TASK_PLAN_SYNCH taskPlanSynchCmd;
    lbvTaskQueueCommand(&taskPlanSynchCmd);

    // without lbvTaskPlanClose(), a new run command resumes at
    // aborted line-- feature that may be considered later
    {
	int was_open = taskplanopen;
	lbvTaskPlanClose();
	if (lbv_debug & LBV_DEBUG_INTERP && was_open) {
	    rcs_print("lbvTaskPlanClose() called at %s:%d\n", __FILE__,
		      __LINE__);
	}
    }

    return 0;
}

int lbvTaskSetMode(int mode)
{
    int retval = 0;

    switch (mode) {
    case LBV_TASK_MODE_MANUAL:
	// go to manual mode
        if (all_homed()) {
            lbvTrajSetMode(LBV_TRAJ_MODE_TELEOP);
        } else {
            lbvTrajSetMode(LBV_TRAJ_MODE_FREE);
        }
	mdiOrAuto = LBV_TASK_MODE_AUTO;	// we'll default back to here
	break;

    case LBV_TASK_MODE_MDI:
	// go to mdi mode
	lbvTrajSetMode(LBV_TRAJ_MODE_COORD);
	lbvTaskAbort();
	lbvTaskPlanSynch();
	mdiOrAuto = LBV_TASK_MODE_MDI;
	break;

    case LBV_TASK_MODE_AUTO:
	// go to auto mode
	lbvTrajSetMode(LBV_TRAJ_MODE_COORD);
	lbvTaskAbort();
	lbvTaskPlanSynch();
	mdiOrAuto = LBV_TASK_MODE_AUTO;
	break;

    default:
	retval = -1;
	break;
    }

    return retval;
}

int lbvTaskSetState(int state)
{
    int t;
    int retval = 0;

    switch (state) {
    case LBV_TASK_STATE_OFF:
        lbvMotionAbort();
	// turn the machine servos off-- go into READY state
    for (t = 0; t < lbvStatus->motion.traj.spindles; t++)  lbvSpindleAbort(t);
	for (t = 0; t < lbvStatus->motion.traj.joints; t++) {
	    lbvJointDisable(t);
	}
	lbvTrajDisable();
	lbvIoAbort(LBV_ABORT_TASK_STATE_OFF);
	lbvLubeOff();
	lbvTaskAbort();
    lbvJointUnhome(-2); // only those joints which are volatile_home
	lbvAbortCleanup(LBV_ABORT_TASK_STATE_OFF);
	lbvTaskPlanSynch();
	break;

    case LBV_TASK_STATE_ON:
	// turn the machine servos on
	lbvTrajEnable();
	for (t = 0; t < lbvStatus->motion.traj.joints; t++){
		lbvJointEnable(t);
	}
	lbvLubeOn();
	break;

    case LBV_TASK_STATE_ESTOP_RESET:
	// reset the estop
	lbvAuxEstopOff();
	lbvLubeOff();
	lbvTaskAbort();
        lbvIoAbort(LBV_ABORT_TASK_STATE_ESTOP_RESET);
    for (t = 0; t < lbvStatus->motion.traj.spindles; t++) lbvSpindleAbort(t);
	lbvAbortCleanup(LBV_ABORT_TASK_STATE_ESTOP_RESET);
	lbvTaskPlanSynch();
	break;

    case LBV_TASK_STATE_ESTOP:
        lbvMotionAbort();
	for (t = 0; t < lbvStatus->motion.traj.spindles; t++) lbvSpindleAbort(t);
	// go into estop-- do both IO estop and machine servos off
	lbvAuxEstopOn();
	for (t = 0; t < lbvStatus->motion.traj.joints; t++) {
	    lbvJointDisable(t);
	}
	lbvTrajDisable();
	lbvLubeOff();
	lbvTaskAbort();
        lbvIoAbort(LBV_ABORT_TASK_STATE_ESTOP);
	for (t = 0; t < lbvStatus->motion.traj.spindles; t++) lbvSpindleAbort(t);
        lbvJointUnhome(-2); // only those joints which are volatile_home
	lbvAbortCleanup(LBV_ABORT_TASK_STATE_ESTOP);
	lbvTaskPlanSynch();
	break;

    default:
	retval = -1;
	break;
    }

    return retval;
}

// WM access functions

/*
  determineMode()

  Looks at mode of subsystems, and returns associated mode

  Depends on traj mode, and mdiOrAuto flag

  traj mode   mdiOrAuto     task mode
  ---------   ---------     ---------
  FREE        XXX           MANUAL
  TELEOP      XXX           MANUAL
  COORD       MDI           MDI
  COORD       AUTO          AUTO
  */
static int determineMode()
{
    if (lbvStatus->motion.traj.mode == LBV_TRAJ_MODE_FREE) {
        return LBV_TASK_MODE_MANUAL;
    }
    if (lbvStatus->motion.traj.mode == LBV_TRAJ_MODE_TELEOP) {
        return LBV_TASK_MODE_MANUAL;
    }
    // for LBV_TRAJ_MODE_COORD
    return mdiOrAuto;
}

/*
  determineState()

  Looks at state of subsystems, and returns associated state

  Depends on traj enabled, io estop, and desired task state

  traj enabled   io estop      state
  ------------   --------      -----
  DISABLED       ESTOP         ESTOP
  ENABLED        ESTOP         ESTOP
  DISABLED       OUT OF ESTOP  ESTOP_RESET
  ENABLED        OUT OF ESTOP  ON
  */
static int determineState()
{
    if (lbvStatus->io.aux.estop) {
	return LBV_TASK_STATE_ESTOP;
    }

    if (!lbvStatus->motion.traj.enabled) {
	return LBV_TASK_STATE_ESTOP_RESET;
    }

    return LBV_TASK_STATE_ON;
}

static int waitFlag = 0;

static char interp_error_text_buf[LINELEN];
static char interp_stack_buf[LINELEN];

static void print_interp_error(int retval)
{
    int index = 0;
    if (retval == 0) {
	return;
    }

    if (0 != lbvStatus) {
	lbvStatus->task.interpreter_errcode = retval;
    }

    interp_error_text_buf[0] = 0;
    interp.error_text(retval, interp_error_text_buf, LINELEN);
    if (0 != interp_error_text_buf[0]) {
	rcs_print_error("interp_error: %s\n", interp_error_text_buf);
    }
    lbvOperatorError(0, "%s", interp_error_text_buf);
    index = 0;
    if (lbv_debug & LBV_DEBUG_INTERP) {
	rcs_print("Interpreter stack: \t");
	while (index < 5) {
	    interp_stack_buf[0] = 0;
	    interp.stack_name(index, interp_stack_buf, LINELEN);
	    if (0 == interp_stack_buf[0]) {
		break;
	    }
	    rcs_print(" - %s ", interp_stack_buf);
	    index++;
	}
	rcs_print("\n");
    }
}

int lbvTaskPlanInit()
{
    if(!pinterp) {
	IniFile inifile;
	const char *inistring;
	inifile.Open(lbv_inifile);
	if((inistring = inifile.Find("INTERPRETER", "TASK"))) {
	    pinterp = interp_from_shlib(inistring);
	    fprintf(stderr, "interp_from_shlib() -> %p\n", pinterp);
	}
        inifile.Close();
    }
    if(!pinterp) {
        pinterp = new Interp;
    }

    Interp *i = dynamic_cast<Interp*>(pinterp);
    if(i) _is = &i->_setup; // FIXME
    else  _is = 0;
    interp.ini_load(lbv_inifile);
    waitFlag = 0;

    int retval = interp.init();
    // In task, enable M99 main program endless looping
    interp.set_loop_on_main_m99(true);
    if (retval > INTERP_MIN_ERROR) {  // I'd think this should be fatal.
	print_interp_error(retval);
    } else {
	if (0 != rs274ngc_startup_code[0]) {
	    retval = interp.execute(rs274ngc_startup_code);
	    while (retval == INTERP_EXECUTE_FINISH) {
		retval = interp.execute(0);
	    }
	    if (retval > INTERP_MIN_ERROR) {
		print_interp_error(retval);
	    }
	}
    }

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanInit() returned %d\n", retval);
    }

    return retval;
}

int lbvTaskPlanSetWait()
{
    waitFlag = 1;

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanSetWait() called\n");
    }

    return 0;
}

int lbvTaskPlanIsWait()
{
    return waitFlag;
}

int lbvTaskPlanClearWait()
{
    waitFlag = 0;

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanClearWait() called\n");
    }

    return 0;
}

int lbvTaskPlanSetOptionalStop(bool state)
{
    SET_OPTIONAL_PROGRAM_STOP(state);
    return 0;
}

int lbvTaskPlanSetBlockDelete(bool state)
{
    SET_BLOCK_DELETE(state);
    return 0;
}


int lbvTaskPlanSynch()
{
    int retval = interp.synch();
    if (retval == INTERP_ERROR) {
        lbvTaskAbort();
    }

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanSynch() returned %d\n", retval);
    }

    return retval;
}

void lbvTaskPlanExit()
{
    if (pinterp != NULL) {
        interp.exit();
    }
}

int lbvTaskPlanOpen(const char *file)
{
    if (lbvStatus != 0) {
	lbvStatus->task.motionLine = 0;
	lbvStatus->task.currentLine = 0;
	lbvStatus->task.readLine = 0;
    }

    int retval = interp.open(file);
    if (retval > INTERP_MIN_ERROR) {
	print_interp_error(retval);
	return retval;
    }
    taskplanopen = 1;

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanOpen(%s) returned %d\n", file, retval);
    }

    return retval;
}


int lbvTaskPlanRead()
{
    int retval = interp.read();
    if (retval == INTERP_FILE_NOT_OPEN) {
	if (lbvStatus->task.file[0] != 0) {
	    retval = interp.open(lbvStatus->task.file);
	    if (retval > INTERP_MIN_ERROR) {
		print_interp_error(retval);
	    }
	    retval = interp.read();
	}
    }
    if (retval > INTERP_MIN_ERROR) {
	print_interp_error(retval);
    }
    
    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanRead() returned %d\n", retval);
    }
    
    return retval;
}

int lbvTaskPlanExecute(const char *command)
{
    int inpos = lbvStatus->motion.traj.inpos;	// 1 if in position, 0 if not.

    if (command != 0) {		// Command is 0 if in AUTO mode, non-null if in MDI mode.
	// Don't sync if not in position.
	if ((*command != 0) && (inpos)) {
	    interp.synch();
	}
    }
    int retval = interp.execute(command);
    if (retval > INTERP_MIN_ERROR) {
	print_interp_error(retval);
    }
    if(command != 0) {
	FINISH();
    }

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanExecute(0) return %d\n", retval);
    }

    return retval;
}

int lbvTaskPlanExecute(const char *command, int line_number)
{
    int retval = interp.execute(command, line_number);
    if (retval > INTERP_MIN_ERROR) {
	print_interp_error(retval);
    }
    if(command != 0) { // this means MDI
	FINISH();
    }

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanExecute(%s) returned %d\n", command, retval);
    }

    return retval;
}

int lbvTaskPlanClose()
{
    int retval = interp.close();
    if (retval > INTERP_MIN_ERROR) {
	print_interp_error(retval);
    }

    taskplanopen = 0;
    return retval;
}

int lbvTaskPlanReset()
{
    int retval = interp.reset();
    if (retval > INTERP_MIN_ERROR) {
	print_interp_error(retval);
    }

    return retval;
}

int lbvTaskPlanLine()
{
    int retval = interp.line();
    
    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanLine() returned %d\n", retval);
    }

    return retval;
}

int lbvTaskPlanLevel()
{
    int retval = interp.call_level();

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanLevel() returned %d\n", retval);
    }

    return retval;
}

int lbvTaskPlanCommand(char *cmd)
{
    char buf[LINELEN];

    strcpy(cmd, interp.command(buf, LINELEN));

    if (lbv_debug & LBV_DEBUG_INTERP) {
        rcs_print("lbvTaskPlanCommand(%s) called. (line_number=%d)\n",
          cmd, lbvStatus->task.readLine);
    }

    return 0;
}

int lbvTaskUpdate(LBV_TASK_STAT * stat)
{
    stat->mode = (enum LBV_TASK_MODE_ENUM) determineMode();
    int oldstate = stat->state;
    stat->state = (enum LBV_TASK_STATE_ENUM) determineState();

    if(oldstate == LBV_TASK_STATE_ON && oldstate != stat->state) {
	lbvTaskAbort();
    for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);
        lbvIoAbort(LBV_ABORT_TASK_STATE_NOT_ON);
	lbvAbortCleanup(LBV_ABORT_TASK_STATE_NOT_ON);
    }

    // execState set in main
    // interpState set in main
    if (lbvStatus->motion.traj.id > 0) {
	stat->motionLine = lbvStatus->motion.traj.id;
    }
    // currentLine set in main
    // readLine set in main

    char buf[LINELEN];
    strcpy(stat->file, interp.file(buf, LINELEN));
    // command set in main

    // update active G and M codes
    interp.active_g_codes(&stat->activeGCodes[0]);
    interp.active_m_codes(&stat->activeMCodes[0]);
    interp.active_settings(&stat->activeSettings[0]);

    //update state of optional stop
    stat->optional_stop_state = GET_OPTIONAL_PROGRAM_STOP();
    
    //update state of block delete
    stat->block_delete_state = GET_BLOCK_DELETE();
    
    stat->heartbeat++;

    return 0;
}

int lbvAbortCleanup(int reason, const char *message)
{
    int status = interp.on_abort(reason,message);
    if (status > INTERP_MIN_ERROR)
	print_interp_error(status);
    return status;
}

