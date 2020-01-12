/********************************************************************
* Description: lbvsh.cc
*   Extended-Tcl-based LBV automatic test interface
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
*         Reorganized by Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <tcl.h>
#include <tk.h>

#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"		// LBV NML
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "lbvglb.h"		// LBV_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "lbvcfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "rcs_print.hh"
#include "timer.hh"

#include "shcom.hh"

#define setresult(t,s) Tcl_SetObjResult((t), Tcl_NewStringObj((s),-1))

/*
  Using tcl package Labvcnc:
  Using lbvsh:

  % package require Labvcnc
  % lbv_init -ini inifilename # to start with an inifile
  or
  % lbv_init # to start with the default inifilename (lbv.ini)

  With filename, it opens NML buffers to the LBV, runs the script, closes
  the buffers, and quits.

  With -ini <inifile>, uses inifile instead of lbv.ini.

  Commands in the Labvcnc package are all prefixed with "lbv_", which makes them
  somewhat inconvenient for typing but avoids name conflicts, e.g., open.

  Some commands take 0 or more arguments. 0 arguments means they return
  the associated value; the argument would be to set the value.

  Commands are sent to the LBV, and control resumes immediately. You can
  call a timed wait until the command got there, or a timed wait until the
  command completed, or not wait at all.

  LBV commands:

  lbv_plat
  Returns the platform for which this was compiled, e.g., linux_2_0_36

  lbv_ini <var> <section>
  Returns the string value of <var> in section <section>, in LBV_INIFILE

  lbv_debug {<new value>}
  With no arg, returns the integer value of LBV_DEBUG, in the LBV. Note that
  it may not be true that the local LBV_DEBUG variable here (in lbvsh and
  the GUIs that use it) is the same as the LBV_DEBUG value in the LBV. This
  can happen if the LBV is started from one .ini file, and the GUI is started
  with another that has a different value for DEBUG.
  With an arg, sends a command to the LBV to set the new debug level,
  and sets the LBV_DEBUG global here to the same value. This will make
  the two values the same, since they really ought to be the same.

  lbv_set_wait none | received | done
  Set the wait for commands to return to be right away (none), after the
  command was sent and received (received), or after the command was
  done (done).

  lbv_wait received | done
  Force a wait for the previous command to be received, or done. This lets
  you wait in the event that "lbv_set_wait none" is in effect.

  lbv_set_timeout <timeout>
  Set the timeout for commands to return to <timeout>, in seconds. Timeout
  is a real number. If it's <= 0.0, it means wait forever. Default is 0.0,
  wait forever.

  lbv_update (none) | none | auto
  With no arg, forces an update of the LBV status. With "none", doesn't
  cause an automatic update of status with other lbv_ words. With "auto",
  makes lbv_ words automatically update status before they return values.

  lbv_error
  Returns the current LBV error string, or "ok" if no error.

  lbv_operator_display
  Returns the current LBV operator display string, or "ok" if none.

  lbv_operator_text
  Returns the current LBV operator text string, or "ok" if none.

  lbv_time
  Returns the time, in seconds, from the start of the epoch. This starting
  time depends on the platform.

  lbv_estop (none) | on | off
  With no arg, returns the estop setting as "on" or "off". Otherwise,
  sends an estop on or off command.

  lbv_machine (none) | on | off
  With no arg, returns the machine setting as "on" or "off". Otherwise,
  sends a machine on or off command.

  lbv_mode (none) | manual | auto | mdi
  With no arg, returns the mode setting as "manual", "auto", or "mdi".
  Otherwise, sends a mode manual, auto, or mdi command.

  lbv_mist (none) | on | off
  With no arg, returns the mist setting as "on" or "off". Otherwise,
  sends a mist on or off command.

  lbv_flood (none) | on | off
  With no arg, returns the flood setting as "on" or "off". Otherwise,
  sends a flood on or off command.

  lbv_lube (none) | on | off
  With no arg, returns the lubricant pump setting as "on" or "off".
  Otherwise, sends a lube on or off command.

  lbv_lube_level
  Returns the lubricant level sensor reading as "ok" or "low".

  lbv_spindle (spindle_number) (none) | forward | reverse | increase | decrease | constant | off
  With no spindle_number defaults to spindle 0. This is a little different
  from the default behaviour elsewhere where specifyin no spindle affects all spindles.
  With no arg, returns the value of the spindle state as "forward",
  "reverse", "increase", "decrease", or "off". With arg, sends the spindle
  command. Note that "increase" and "decrease" will cause a speed change in
  the corresponding direction until a "constant" command is sent.

  lbv_brake (none) | on | off
  With no arg, returns the brake setting. Otherwise sets the brake.

  lbv_tool
  Returns the id of the currently loaded tool

  lbv_tool_offset X | Y | ...
  Returns the currently applied tool length offset

  lbv_load_tool_table <file>
  Loads the tool table specified by <file>

  lbv_home 0 | 1 | 2 | ...
  Homes the indicated joint.

  lbv_unhome 0 | 1 | 2 | ...
  Unhomes the indicated joint.

  lbv_jog_stop 0 | 1 | 2 | ...
  Stop the joint jog

  lbv_jog 0 | 1 | 2 | ... <speed>
  Jog the indicated joint at <speed>; sign of speed is direction

  lbv_jog_incr 0 | 1 | 2 | ... <speed> <incr>
  Jog the indicated joint by increment <incr> at the <speed>; sign of
  speed is direction

  lbv_feed_override {<percent>}
  With no args, returns the current feed override, as a percent. With
  argument, set the feed override to be the percent value

  lbv_rapid_override {<percent>}
  With no args, returns the current rapid override, as a percent. With
  argument, set the rapid override to be the percent value

  lbv_spindle_override {<percent>}
  With no args, returns the current spindle override, as a percent. With
  argument, set the spindle override to be the percent value

  lbv_abs_cmd_pos X | Y | ...
  Returns double obj containing the commanded pos in abs coords,
  at given index, 0 = X, etc.

  lbv_abs_act_pos X | Y | ...
  Returns double objs containing the actual pos in abs coords

  lbv_rel_cmd_pos  X | Y | ...
  Returns double obj containing the commanded pos in rel coords,
  including tool length offset

  lbv_rel_act_pos  X | Y | ...
  Returns double objs containing the actual pos in rel coords,
  including tool length offset

  lbv_joint_pos
  Returns double objs containing the actual pos in absolute coords of individual
  joint/slider positions, excludes tool length offset

  lbv_pos_offset X | Y | Z | A | B | C | U | V | W
  Returns the position offset associated with the world coordinate provided

  lbv_joint_limit 0 | 1 | ...
  Returns "ok", "minsoft", "minhard", "maxsoft", "maxhard"

  lbv_joint_fault 0 | 1 | ...
  Returns "ok" or "fault"

  lbv_joint_homed 0 | 1 | ...
  Returns "homed", "not"

  lbv_mdi <string>
  Sends the <string> as an MDI command

  lbv_task_plan_init
  Initializes the program interpreter

  lbv_open <filename>
  Opens the named file

  lbv_run {<start line>}
  Without start line, runs the opened program from the beginning. With
  start line, runs from that line. A start line of -1 runs in verify mode.

  lbv_pause
  Pause program execution

  lbv_resume
  Resume program execution

  lbv_step
  Step the program one line

  lbv_program
  Returns the name of the currently opened program, or "none"

  lbv_program_line
  Returns the currently executing line of the program

  lbv_program_status
  Returns "idle", "running", or "paused"

  lbv_program_codes
  Returns the string for the currently active program codes

  lbv_override_limit none | 0 | 1
  returns state of override, sets it or deactivates it (used to jog off hardware limit switches)
  
  lbv_optional_stop  none | 0 | 1
  returns state of optional setop, sets it or deactivates it (used to stop/continue on M1)

  lbv_program_codes
  Returns the string for the currently active program codes

  lbv_joint_type <joint>
  Returns "linear", "angular", or "custom" for the type of the specified joint

  lbv_joint_units <joint>
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the corresponding native units of the specified joint. The type
  of the joint (linear or angular) is used to resolve which type of units
  are returned. The units are obtained heuristically, based on the
  LBV_JOINT_STAT::units numerical value of user units per mm or deg.
  For linear joints, something close to 0.03937 is deemed "inch",
  1.000 is "mm", 0.1 is "cm", otherwise it's "custom".
  For angular joints, something close to 1.000 is deemed "deg",
  PI/180 is "rad", 100/90 is "grad", otherwise it's "custom".
 
  lbv_program_units
  lbv_program_linear_units
  Returns "inch", "mm", "cm", or "none", for the corresponding linear 
  units that are active in the program interpreter.

  lbv_program_angular_units
  Returns "deg", "rad", "grad", or "none" for the corresponding angular
  units that are active in the program interpreter.

  lbv_user_linear_units
  Returns "inch", "mm", "cm", or "custom", for the
  corresponding native user linear units of the LBV trajectory
  level. This is obtained heuristically, based on the
  LBV_TRAJ_STAT::linearUnits numerical value of user units per mm.
  Something close to 0.03937 is deemed "inch", 1.000 is "mm", 0.1 is
  "cm", otherwise it's "custom".

  lbv_user_angular_units
  Returns "deg", "rad", "grad", or "custom" for the corresponding native
  user angular units of the LBV trajectory level. Like with linear units,
  this is obtained heuristically.

  lbv_display_linear_units
  lbv_display_angular_units
  Returns "inch", "mm", "cm", or "deg", "rad", "grad", or "custom",
  for the linear or angular units that are active in the display. 
  This is effectively the value of linearUnitConversion or
  angularUnitConversion, resp.

  lbv_linear_unit_conversion {inch | mm | cm | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.
 
  lbv_angular_unit_conversion {deg | rad | grad | auto}
  With no args, returns the unit conversion active. With arg, sets the
  units to be displayed. If it's "auto", the units to be displayed match
  the program units.

  lbv_probe_clear
  Clear the probe tripped flag.

  lbv_probe_tripped
  Has the probe been tripped since the last clear.

  lbv_probe_value
  Value of current probe signal. (read-only)

  lbv_probe
  Move toward a certain location. If the probe is tripped on the way stop
  motion, record the position and raise the probe tripped flag.

  lbv_teleop_enable
  Should motion run in teleop mode? (No args
  gets it, one arg sets it.)

  lbv_kinematics_type
  returns the type of kinematics functions used identity=1, serial=2,
  parallel=3, custom=4
*/

#define CHECKLBV \
    if (!checkStatus() ) {\
        setresult(interp,"lbv not connected");\
        return TCL_ERROR;\
    }

static void thisQuit(ClientData clientData)
{
    LBV_NULL lbv_null_msg;

    if (0 != lbvStatusBuffer) {
	// wait until current message has been received
	lbvCommandWaitReceived();
    }

    // clean up NML buffers

    if (lbvErrorBuffer != 0) {
	delete lbvErrorBuffer;
	lbvErrorBuffer = 0;
    }

    if (lbvStatusBuffer != 0) {
	delete lbvStatusBuffer;
	lbvStatusBuffer = 0;
	lbvStatus = 0;
    }

    if (lbvCommandBuffer != 0) {
	delete lbvCommandBuffer;
	lbvCommandBuffer = 0;
    }

    return;
}


/* LBV command functions */

static int lbv_plat(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    if (objc == 1) {
	setresult(interp,"Linux");
	return TCL_OK;
    }

    setresult(interp,"lbv_plat: need no args");
    return TCL_ERROR;
}

static int lbv_ini(ClientData clientdata,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    IniFile inifile;
    const char *inistring;
    const char *varstr, *secstr, *defaultstr;
    defaultstr = 0;

    if (objc != 3 && objc != 4) {
	setresult(interp,"lbv_ini: need 'var' and 'section'");
	return TCL_ERROR;
    }
    // open it
    if (inifile.Open(lbv_inifile) == false) {
	return TCL_OK;
    }

    varstr = Tcl_GetStringFromObj(objv[1], 0);
    secstr = Tcl_GetStringFromObj(objv[2], 0);

    if (objc == 4) {
	defaultstr = Tcl_GetStringFromObj(objv[3], 0);
    }

    if (NULL == (inistring = inifile.Find(varstr, secstr))) {
	if (defaultstr != 0) {
	    setresult(interp,(char *) defaultstr);
	}
	return TCL_OK;
    }

    setresult(interp,(char *) inistring);

    // close it
    inifile.Close();

    return TCL_OK;
}

static int lbv_Debug(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    Tcl_Obj *debug_obj;
    int debug;

    CHECKLBV
    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (objc == 1) {
	// no arg-- return current value
	debug_obj = Tcl_NewIntObj(lbvStatus->debug);
	Tcl_SetObjResult(interp, debug_obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (0 != Tcl_GetIntFromObj(0, objv[1], &debug)) {
	    setresult(interp,"lbv_debug: need debug level as integer");
	    return TCL_ERROR;
	}
	sendDebug(debug);
	lbv_debug = debug;
	return TCL_OK;
    }
    // wrong number of args
    setresult(interp,"lbv_debug: need zero or one arg");
    return TCL_ERROR;
}

static int lbv_set_wait(ClientData clientdata,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	switch (lbvWaitType) {
	case LBV_WAIT_RECEIVED:
	    setresult(interp,"received");
	    break;
	case LBV_WAIT_DONE:
	    setresult(interp,"done");
	    break;
	default:
	    setresult(interp,"(invalid)");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "received")) {
	    lbvWaitType = LBV_WAIT_RECEIVED;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "done")) {
	    lbvWaitType = LBV_WAIT_DONE;
	    return TCL_OK;
	}
    }

    setresult(interp, "lbv_set_wait: need 'received', 'done', or no args");
    return TCL_ERROR;
}

static int lbv_wait(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "received")) {
	    if (0 != lbvCommandWaitReceived()) {
		setresult(interp,"timeout");
	    }
	    return TCL_OK;
	}
	if (!strcmp(objstr, "done")) {
	    if (0 != lbvCommandWaitDone()) {
		setresult(interp,"timeout");
	    }
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_wait: need 'received' or 'done'");
    return TCL_ERROR;
}

static int lbv_set_timeout(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    double timeout;
    Tcl_Obj *timeout_obj;

    CHECKLBV
    if (objc == 1) {
	timeout_obj = Tcl_NewDoubleObj(lbvTimeout);
	Tcl_SetObjResult(interp, timeout_obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (TCL_OK == Tcl_GetDoubleFromObj(0, objv[1], &timeout)) {
	    lbvTimeout = timeout;
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_set_timeout: need time as real number");
    return TCL_ERROR;
}

static int lbv_update(ClientData clientdata,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	updateStatus();
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "none")) {
	    lbvUpdateType = LBV_UPDATE_NONE;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    lbvUpdateType = LBV_UPDATE_AUTO;
	    return TCL_OK;
	}
    }

    return TCL_OK;
}

static int lbv_time(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc == 1) {
	Tcl_SetObjResult(interp, Tcl_NewDoubleObj(etime()));
	return TCL_OK;
    }

    setresult(interp,"lbv_time: needs no arguments");
    return TCL_ERROR;
}

static int lbv_error(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{

    CHECKLBV
    if (objc == 1) {
	// get any new error, it's saved in global error_string[]
	if (0 != updateError()) {
	    setresult(interp,"lbv_error: bad status from LBV");
	    return TCL_ERROR;
	}
	// put error on result list
	if (error_string[0] == 0) {
	    setresult(interp,"ok");
	} else {
	    setresult(interp,error_string);
	    error_string[0] = 0;
	}
	return TCL_OK;
    }

    setresult(interp,"lbv_error: need no args");
    return TCL_ERROR;
}

static int lbv_operator_text(ClientData clientdata,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{

    CHECKLBV
    if (objc == 1) {
	// get any new string, it's saved in global operator_text_string[]
	if (0 != updateError()) {
	    setresult(interp,"lbv_operator_text: bad status from LBV");
	    return TCL_ERROR;
	}
	// put error on result list
	if (operator_text_string[0] == 0) {
	    setresult(interp,"ok");
	    operator_text_string[0] = 0;
	} else {
	    setresult(interp,operator_text_string);
	}
	return TCL_OK;
    }

    setresult(interp,"lbv_operator_text: need no args");
    return TCL_ERROR;
}

static int lbv_operator_display(ClientData clientdata,
				Tcl_Interp * interp, int objc,
				Tcl_Obj * CONST objv[])
{

    CHECKLBV
    if (objc == 1) {
	// get any new string, it's saved in global operator_display_string[]
	if (0 != updateError()) {
	    setresult(interp,"lbv_operator_display: bad status from LBV");
	    return TCL_ERROR;
	}
	// put error on result list
	if (operator_display_string[0] == 0) {
	    setresult(interp,"ok");
	} else {
	    setresult(interp,operator_display_string);
	    operator_display_string[0] = 0;
	}
	return TCL_OK;
    }

    setresult(interp,"lbv_operator_display: need no args");
    return TCL_ERROR;
}

static int lbv_estop(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	if (lbvStatus->task.state == LBV_TASK_STATE_ESTOP) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendEstop();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendEstopReset();
	    return TCL_OK;
	}
    }

    setresult(interp, "lbv_estop: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int lbv_machine(ClientData clientdata,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	if (lbvStatus->task.state == LBV_TASK_STATE_ON) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendMachineOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendMachineOff();
	    return TCL_OK;
	}
    }

    setresult(interp, "lbv_machine: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int lbv_mode(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	switch (lbvStatus->task.mode) {
	case LBV_TASK_MODE_MANUAL:
	    setresult(interp,"manual");
	    break;
	case LBV_TASK_MODE_AUTO:
	    setresult(interp,"auto");
	    break;
	case LBV_TASK_MODE_MDI:
	    setresult(interp,"mdi");
	    break;
	default:
	    setresult(interp,"?");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "manual")) {
	    sendManual();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    sendAuto();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "mdi")) {
	    sendMdi();
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_mode: need 'manual', 'auto', 'mdi', or no args");
    return TCL_ERROR;
}

static int lbv_mist(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	if (lbvStatus->io.coolant.mist == 1) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendMistOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendMistOff();
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_mist: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int lbv_flood(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	if (lbvStatus->io.coolant.flood == 1) {
	    setresult(interp,"on");
	} else {
	    setresult(interp,"off");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendFloodOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendFloodOff();
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_flood: need 'on', 'off', or no args"); return TCL_ERROR;
}

static int lbv_lube(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	if (lbvStatus->io.lube.on == 0) {
	    setresult(interp,"off");
	} else {
	    setresult(interp,"on");
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "on")) {
	    sendLubeOn();
	    return TCL_OK;
	}
	if (!strcmp(objstr, "off")) {
	    sendLubeOff();
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_lube: need 'on', 'off', or no args");
    return TCL_ERROR;
}

static int lbv_lube_level(ClientData clientdata,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	if (lbvStatus->io.lube.level == 0) {
	    setresult(interp,"low");
	} else {
	    setresult(interp,"ok");
	}
	return TCL_OK;
    }

    setresult(interp,"lbv_lube_level: need no args");
    return TCL_ERROR;
}

static int lbv_spindle(ClientData clientdata,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    char *objstr = NULL;
    int spindle = 0;

    CHECKLBV

    if (objc >= 2) {
        if (Tcl_GetIntFromObj(interp, objv[1], &spindle) != TCL_OK){ // not a likely spindle index first, then
            spindle = 0;
            objstr = Tcl_GetStringFromObj(objv[1], 0);
        } else {
            if (spindle < 0 || spindle > LBVMOT_MAX_SPINDLES){ // should really be num_spindles, but not sure we know that here
                setresult(interp,"invalid spindle index number");
                return TCL_ERROR;
            }
            objstr = Tcl_GetStringFromObj(objv[2], 0);
        }
    }
    if (objstr) {
        if (!strcmp(objstr, "forward")) {
            sendSpindleForward(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "reverse")) {
            sendSpindleReverse(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "increase")) {
            sendSpindleIncrease(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "decrease")) {
            sendSpindleDecrease(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "constant")) {
            sendSpindleConstant(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "off")) {
            sendSpindleOff(spindle);
            return TCL_OK;
        }
    }
    else
    {
    // no arg-- return status
        if (lbvUpdateType == LBV_UPDATE_AUTO) {
            updateStatus();
        }
        if (lbvStatus->motion.spindle[spindle].increasing > 0) {
            setresult(interp,"increase");
        } else if (lbvStatus->motion.spindle[spindle].increasing < 0) {
            setresult(interp,"decrease");
        } else if (lbvStatus->motion.spindle[spindle].direction > 0) {
            setresult(interp,"forward");
        } else if (lbvStatus->motion.spindle[spindle].direction < 0) {
            setresult(interp,"reverse");
        } else {
            setresult(interp,"off");
        }
        return TCL_OK;
    }
    setresult(interp,"lbv_spindle: need 'on', 'off', a spindle index or no args");
    return TCL_ERROR;
}

static int lbv_brake(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char *objstr = NULL;
    int spindle = 0;

    CHECKLBV

    if (objc >= 2) {
        if (Tcl_GetIntFromObj(interp, objv[1], &spindle) != TCL_OK){ // not a likely spindle index first, then
            spindle = 0;
            objstr = Tcl_GetStringFromObj(objv[1], 0);
        } else {
            if (spindle < 0 || spindle > LBVMOT_MAX_SPINDLES){ // FIXME: should really be num_spindles, but not sure we know that here
                setresult(interp,"invalid spindle index number");
                return TCL_ERROR;
            }
            objstr = Tcl_GetStringFromObj(objv[2], 0);
        }
    }

    if (objstr) {
        if (!strcmp(objstr, "on")) {
            sendBrakeEngage(spindle);
            return TCL_OK;
        }
        if (!strcmp(objstr, "off")) {
            sendBrakeRelease(spindle);
            return TCL_OK;
        }
    }
    else
    {
        // no arg-- return status
        if (lbvUpdateType == LBV_UPDATE_AUTO) {
            updateStatus();
        }
        if (lbvStatus->motion.spindle[spindle].brake == 1) {
            setresult(interp,"on");
        } else {
            setresult(interp,"off");
        }
    return TCL_OK;
    }
    setresult(interp,"lbv_brake: need 'on', 'off', spindle index or no args");
    return TCL_ERROR;
}

static int lbv_tool(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    Tcl_Obj *toolobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_tool: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    toolobj = Tcl_NewIntObj(lbvStatus->io.tool.toolInSpindle);

    Tcl_SetObjResult(interp, toolobj);
    return TCL_OK;
}

static int lbv_tool_offset(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *tlobj;
    string[0] = 'Z'; //default if not specified

    CHECKLBV
    if (objc > 2) {
	setresult(interp,"lbv_tool_offset: need 0 or 1 args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (objc != 1) {
       strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);
    }

    switch (string[0]) {
    case 'x': case 'X':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                lbvStatus->task.toolOffset.tran.x));
        break;
    case 'y': case 'Y':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                lbvStatus->task.toolOffset.tran.y));
        break;
    case 'z': case 'Z':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                lbvStatus->task.toolOffset.tran.z));
        break;
    case 'a': case 'A':
        tlobj = Tcl_NewDoubleObj(convertAngularUnits(
                                 lbvStatus->task.toolOffset.a));
        break;
    case 'b': case 'B':
        tlobj = Tcl_NewDoubleObj(convertAngularUnits(
                                 lbvStatus->task.toolOffset.b));
        break;
    case 'c': case 'C':
        tlobj = Tcl_NewDoubleObj(convertAngularUnits(
                                 lbvStatus->task.toolOffset.c));
        break;
    case 'u': case 'U':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                 lbvStatus->task.toolOffset.u));
        break;
    case 'v': case 'V':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                 lbvStatus->task.toolOffset.v));
        break;
    case 'w': case 'W':
        tlobj = Tcl_NewDoubleObj(convertLinearUnits(
                                 lbvStatus->task.toolOffset.w));
        break;
    default:
        setresult(interp,"lbv_tool_offset: bad coordinate letter argument");
        return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, tlobj);
    return TCL_OK;
}

static int lbv_load_tool_table(ClientData clientdata,
			       Tcl_Interp * interp, int objc,
			       Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_load_tool_table: need file");
	return TCL_ERROR;
    }

    if (0 != sendLoadToolTable(Tcl_GetStringFromObj(objv[1], 0))) {
	setresult(interp,"lbv_load_tool_table: can't open file");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_set_tool_offset(ClientData clientdata,
			       Tcl_Interp * interp, int objc,
			       Tcl_Obj * CONST objv[])
{
    int tool;
    double length;
    double diameter;

    CHECKLBV
    if (objc != 4) {
	setresult(interp,"lbv_set_tool_offset: need <tool> <length> <diameter>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &tool)) {
	setresult(interp,"lbv_set_tool_offset: need tool as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[2], &length)) {
	setresult(interp,"lbv_set_tool_offset: need length as real number");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &diameter)) {
	setresult(interp,"lbv_set_tool_offset: need diameter as real number");
	return TCL_ERROR;
    }

    if (0 != sendToolSetOffset(tool, length, diameter)) {
	setresult(interp,"lbv_set_tool_offset: can't set it");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_abs_cmd_pos(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_abs_cmd_pos: need exactly 1 coordinate letter");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.position.tran.x));
        break;
    case 'y': case 'Y':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.position.tran.y));
        break;
    case 'z': case 'Z':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.position.tran.z));
        break;
    case 'a': case 'A':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.position.a));
        break;
    case 'b': case 'B':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.position.b));
        break;
    case 'c': case 'C':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.position.c));
        break;
    case 'u': case 'U':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.position.u));
        break;
    case 'v': case 'V':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.position.v));
        break;
    case 'w': case 'W':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.position.w));
        break;
    default:
        setresult(interp,"lbv_abs_cmd_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int lbv_abs_act_pos(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_abs_act_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.actualPosition.tran.x));
        break;
    case 'y': case 'Y':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.actualPosition.tran.y));
        break;
    case 'z': case 'Z':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.actualPosition.tran.z));
        break;
    case 'a': case 'A':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.actualPosition.a));
        break;
    case 'b': case 'B':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.actualPosition.b));
        break;
    case 'c': case 'C':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.actualPosition.c));
        break;
    case 'u': case 'U':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.actualPosition.u));
        break;
    case 'v': case 'V':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.actualPosition.v));
        break;
    case 'w': case 'W':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.actualPosition.w));
        break;
    default:
        setresult(interp,"lbv_abs_act_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int lbv_rel_cmd_pos(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_rel_cmd_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    double d = 0.0;
    switch (string[0]) {
    case 'x': case 'X':
        d = convertLinearUnits(lbvStatus->motion.traj.position.tran.x -
                               lbvStatus->task.g5x_offset.tran.x -
                               lbvStatus->task.g92_offset.tran.x -
                               lbvStatus->task.toolOffset.tran.x);
        break;
    case 'y': case 'Y':
        d = convertLinearUnits(lbvStatus->motion.traj.position.tran.y -
                               lbvStatus->task.g5x_offset.tran.y -
                               lbvStatus->task.g92_offset.tran.y -
                               lbvStatus->task.toolOffset.tran.y);
        break;
    case 'z': case 'Z':
        d = convertLinearUnits(lbvStatus->motion.traj.position.tran.z -
                               lbvStatus->task.g5x_offset.tran.z -
                               lbvStatus->task.g92_offset.tran.z -
                               lbvStatus->task.toolOffset.tran.z);
        break;
    case 'a': case 'A':
        d = convertAngularUnits(lbvStatus->motion.traj.position.a -
                                lbvStatus->task.g5x_offset.a -
                                lbvStatus->task.g92_offset.a -
                                lbvStatus->task.toolOffset.a);
        break;
    case 'b': case 'B':
        d = convertAngularUnits(lbvStatus->motion.traj.position.b -
                                lbvStatus->task.g5x_offset.b -
                                lbvStatus->task.g92_offset.b -
                                lbvStatus->task.toolOffset.b);
        break;
    case 'c': case 'C':
        d = convertAngularUnits(lbvStatus->motion.traj.position.c -
                                lbvStatus->task.g5x_offset.c -
                                lbvStatus->task.g92_offset.c -
                                lbvStatus->task.toolOffset.c);
        break;
    case 'u': case 'U':
        d = convertLinearUnits(lbvStatus->motion.traj.position.u -
                               lbvStatus->task.g5x_offset.u -
                               lbvStatus->task.g92_offset.u -
                               lbvStatus->task.toolOffset.u);
        break;
    case 'v': case 'V':
        d = convertLinearUnits(lbvStatus->motion.traj.position.v -
                               lbvStatus->task.g5x_offset.v -
                               lbvStatus->task.g92_offset.v -
                               lbvStatus->task.toolOffset.v);
        break;
    case 'w': case 'W':
        d = convertLinearUnits(lbvStatus->motion.traj.position.w -
                               lbvStatus->task.g5x_offset.w -
                               lbvStatus->task.g92_offset.w -
                               lbvStatus->task.toolOffset.w);
        break;
    default:
        setresult(interp,"lbv_rel_cmd_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }
    posobj = Tcl_NewDoubleObj(d);
    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int lbv_rel_act_pos(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_rel_act_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    double d = 0.0;
    switch (string[0]) {
    case 'x': case 'X':
        d = convertLinearUnits(lbvStatus->motion.traj.actualPosition.tran.x -
                               lbvStatus->task.g5x_offset.tran.x -
                               lbvStatus->task.g92_offset.tran.x -
                               lbvStatus->task.toolOffset.tran.x);
        break;
    case 'y': case 'Y':
        d = convertLinearUnits(lbvStatus->motion.traj.actualPosition.tran.y -
                               lbvStatus->task.g5x_offset.tran.y -
                               lbvStatus->task.g92_offset.tran.y -
                               lbvStatus->task.toolOffset.tran.y);
        break;
    case 'z': case 'Z':
        d = convertLinearUnits(lbvStatus->motion.traj.actualPosition.tran.z -
                               lbvStatus->task.g5x_offset.tran.z -
                               lbvStatus->task.g92_offset.tran.z -
                               lbvStatus->task.toolOffset.tran.z);
        break;
    case 'a': case 'A':
        d = convertAngularUnits(lbvStatus->motion.traj.actualPosition.a -
                                lbvStatus->task.g5x_offset.a -
                                lbvStatus->task.g92_offset.a -
                                lbvStatus->task.toolOffset.a);
        break;
    case 'b': case 'B':
        d = convertAngularUnits(lbvStatus->motion.traj.actualPosition.b -
                                lbvStatus->task.g5x_offset.b -
                                lbvStatus->task.g92_offset.b -
                                lbvStatus->task.toolOffset.b);
        break;
    case 'c': case 'C':
        d = convertAngularUnits(lbvStatus->motion.traj.actualPosition.c -
                                lbvStatus->task.g5x_offset.c -
                                lbvStatus->task.g92_offset.c -
                                lbvStatus->task.toolOffset.c);
        break;
    case 'u': case 'U':
        d = convertLinearUnits(lbvStatus->motion.traj.actualPosition.u -
                               lbvStatus->task.g5x_offset.u -
                               lbvStatus->task.g92_offset.u -
                               lbvStatus->task.toolOffset.u);
        break;
    case 'v': case 'V':
        d = convertLinearUnits(lbvStatus->motion.traj.actualPosition.v -
                               lbvStatus->task.g5x_offset.v -
                               lbvStatus->task.g92_offset.v -
                               lbvStatus->task.toolOffset.v);
        break;
    case 'w': case 'W':
        d = convertLinearUnits(lbvStatus->motion.traj.actualPosition.w -
                               lbvStatus->task.g5x_offset.w -
                               lbvStatus->task.g92_offset.w -
                               lbvStatus->task.toolOffset.w);
        break;
    default:
        setresult(interp,"lbv_rel_act_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }

    posobj = Tcl_NewDoubleObj(d);
    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int lbv_joint_pos(ClientData clientdata,
			 Tcl_Interp * interp, int objc,
			 Tcl_Obj * CONST objv[])
{
    int joint;
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_joint_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	posobj = Tcl_NewDoubleObj(lbvStatus->motion.joint[joint].input);
    } else {
	setresult(interp,"lbv_joint_pos: bad integer argument");
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int lbv_pos_offset(ClientData clientdata,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_pos_offset: need exactly 1 coordinate letter");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(lbvStatus->task.g5x_offset.tran.x
                                                    +lbvStatus->task.g92_offset.tran.x));
        break;
    case 'y': case 'Y':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(lbvStatus->task.g5x_offset.tran.y
                                                    +lbvStatus->task.g92_offset.tran.y));
        break;
    case 'z': case 'Z':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(lbvStatus->task.g5x_offset.tran.z
                                                    +lbvStatus->task.g92_offset.tran.z));
        break;
    case 'a': case 'A':
	posobj = Tcl_NewDoubleObj(convertAngularUnits(lbvStatus->task.g5x_offset.a
                                                    +lbvStatus->task.g92_offset.a));
        break;
    case 'b': case 'B':
	posobj = Tcl_NewDoubleObj(convertAngularUnits(lbvStatus->task.g5x_offset.b
                                                    +lbvStatus->task.g92_offset.b));
        break;
    case 'c': case 'C':
	posobj = Tcl_NewDoubleObj(convertAngularUnits(lbvStatus->task.g5x_offset.c
                                                    +lbvStatus->task.g92_offset.c));
        break;
    case 'u': case 'U':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(lbvStatus->task.g5x_offset.u
                                                    +lbvStatus->task.g92_offset.u));
        break;
    case 'v': case 'V':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(lbvStatus->task.g5x_offset.v
                                                    +lbvStatus->task.g92_offset.v));
        break;
    case 'w': case 'W':
	posobj = Tcl_NewDoubleObj(convertLinearUnits(lbvStatus->task.g5x_offset.w
                                                    +lbvStatus->task.g92_offset.w));
        break;
    default:
	setresult(interp,"lbv_pos_offset: bad coordinate letter argument");
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

static int lbv_joint_limit(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_joint_limit: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	    setresult(interp,"lbv_joint_limit: joint out of bounds");
	    return TCL_ERROR;
	}

	if (lbvStatus->motion.joint[joint].minHardLimit) {
	    setresult(interp,"minhard");
	    return TCL_OK;
	} else if (lbvStatus->motion.joint[joint].minSoftLimit) {
	    setresult(interp,"minsoft");
	    return TCL_OK;
	} else if (lbvStatus->motion.joint[joint].maxSoftLimit) {
	    setresult(interp,"maxsoft");
	    return TCL_OK;
	} else if (lbvStatus->motion.joint[joint].maxHardLimit) {
	    setresult(interp,"maxsoft");
	    return TCL_OK;
	} else {
	    setresult(interp,"ok");
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_joint_limit: joint out of bounds");
    return TCL_ERROR;
}

static int lbv_joint_fault(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_joint_fault: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	    setresult(interp,"lbv_joint_fault: joint out of bounds");
	    return TCL_ERROR;
	}

	if (lbvStatus->motion.joint[joint].fault) {
	    setresult(interp,"fault");
	    return TCL_OK;
	} else {
	    setresult(interp,"ok");
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_joint_fault: joint out of bounds");
    return TCL_ERROR;
}

static int lbv_override_limit(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    Tcl_Obj *obj;
    int on;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	// motion overrides all axes at same time, so just reference index 0
	obj = Tcl_NewIntObj(lbvStatus->motion.joint[0].overrideLimits);
	Tcl_SetObjResult(interp, obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &on)) {
	    if (on) {
		if (0 != sendOverrideLimits(0)) {
		    setresult(interp,"lbv_override_limit: can't send command");
		    return TCL_OK;
		}
	    } else {
		if (0 != sendOverrideLimits(-1)) {
		    setresult(interp,"lbv_override_limit: can't send command");
		    return TCL_OK;
		}
	    }
	    return TCL_OK;
	} else {
	    setresult(interp,"lbv_override_limit: need 0 or 1");
	    return TCL_ERROR;
	}
    }

    setresult(interp,"lbv_override_limit: need no args, 0 or 1");
    return TCL_ERROR;
}

static int lbv_joint_homed(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_joint_homed: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	    setresult(interp,"lbv_joint_homed: joint out of bounds");
	    return TCL_ERROR;
	}

	if (lbvStatus->motion.joint[joint].homed) {
	    setresult(interp,"homed");
	    return TCL_OK;
	} else {
	    setresult(interp,"not");
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_joint_homed: joint out of bounds");
    return TCL_ERROR;
}

static int lbv_mdi(ClientData clientdata,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    char string[256];
    int t;

    CHECKLBV
    if (objc < 2) {
	setresult(interp,"lbv_mdi: need command");
	return TCL_ERROR;
    }
    // bug-- check for string overflow
    strcpy(string, Tcl_GetStringFromObj(objv[1], 0));
    for (t = 2; t < objc; t++) {
	strcat(string, " ");
	strcat(string, Tcl_GetStringFromObj(objv[t], 0));
    }

    if (0 != sendMdiCmd(string)) {
	setresult(interp,"lbv_mdi: error executing command");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_home(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_home: need joint");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	sendHome(joint);
	return TCL_OK;
    }

    setresult(interp,"lbv_home: need joint as integer, 0..");
    return TCL_ERROR;
}

static int lbv_unhome(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_unhome: need joint");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	sendUnHome(joint);
	return TCL_OK;
    }

    setresult(interp,"lbv_unhome: need joint as integer, 0..");
    return TCL_ERROR;
}

static int lbv_jog_stop(ClientData clientdata,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{
    int joint;
    int jjogmode;

    CHECKLBV
    if (objc != 3) {
	setresult(interp,"lbv_jog_stop: need joint,jogmode");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint)) {
	setresult(interp,"lbv_jog_stop: need joint as integer, 0|1");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[2], &jjogmode)) {
	setresult(interp,"lbv_jog_stop: need jogmode as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != sendJogStop(joint, jjogmode)) {
	setresult(interp,"lbv_jog_stop: can't send jog stop msg");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_jog(ClientData clientdata,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int joint;
    int jjogmode;
    double speed;

    CHECKLBV
    if (objc != 4) {
	setresult(interp,"lbv_jog: need joint,jjogmode and speed");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint)) {
	setresult(interp,"lbv_jog: need joint as integer, 0|1");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[2], &jjogmode)) {
	setresult(interp,"lbv_jog: need jogmode as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &speed)) {
	setresult(interp,"lbv_jog: need speed as real number");
	return TCL_ERROR;
    }

    if (0 != sendJogCont(joint, jjogmode, speed)) {
	setresult(interp,"lbv_jog: can't jog");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_jog_incr(ClientData clientdata,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{
    int joint;
    int jjogmode;
    double speed;
    double incr;

    CHECKLBV
    if (objc != 5) {
	setresult(interp,"lbv_jog_incr: need jjogmode,joint, speed, and increment");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint)) {
	setresult(interp,"lbv_jog_incr: need joint as integer, 0|1");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetIntFromObj(0, objv[2], &jjogmode)) {
	setresult(interp,"lbv_jog_incr: need jogmode as integer, 0..");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &speed)) {
	setresult(interp,"lbv_jog_incr: need speed as real number");
	return TCL_ERROR;
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[4], &incr)) {
	setresult(interp,"lbv_jog_incr: need increment as real number");
	return TCL_ERROR;
    }
    if (0 != sendJogIncr(joint, jjogmode, speed, incr)) {
	setresult(interp,"lbv_jog_incr: can't jog");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_feed_override(ClientData clientdata,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *feedobj;
    int percent;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	feedobj =
	    Tcl_NewIntObj((int)
			  (lbvStatus->motion.traj.scale * 100.0 + 0.5));
	Tcl_SetObjResult(interp, feedobj);
	return TCL_OK;
    }

    if (objc != 2) {
	setresult(interp,"lbv_feed_override: need percent");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &percent)) {
	sendFeedOverride(((double) percent) / 100.0);
	return TCL_OK;
    }

    setresult(interp,"lbv_feed_override: need percent");
    return TCL_ERROR;
}

static int lbv_rapid_override(ClientData clientdata,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *rapidobj;
    int percent;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	rapidobj =
	    Tcl_NewIntObj((int)
			  (lbvStatus->motion.traj.rapid_scale * 100.0 + 0.5));
	Tcl_SetObjResult(interp, rapidobj);
	return TCL_OK;
    }

    if (objc != 2) {
	setresult(interp,"lbv_rapid_override: need percent");
	return TCL_ERROR;
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &percent)) {
	sendRapidOverride(((double) percent) / 100.0);
	return TCL_OK;
    }

    setresult(interp,"lbv_rapid_override: need percent");
    return TCL_ERROR;
}

static int lbv_spindle_override(ClientData clientdata,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *feedobj;
    int spindle = 0;
    int percent;

    CHECKLBV
    if (objc == 1) {
		// no arg-- return status
		if (lbvUpdateType == LBV_UPDATE_AUTO) {
			updateStatus();
		}
		feedobj = Tcl_NewIntObj((int)(lbvStatus->motion.spindle[spindle].spindle_scale * 100.0 + 0.5));
		Tcl_SetObjResult(interp, feedobj);
		return TCL_OK;
    }

    if (objc == 2){
		if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &percent)) {
		sendSpindleOverride(spindle, ((double) percent) / 100.0);
		return TCL_OK;
		}
    }

    if (objc == 3){ // spindle number included
		if (TCL_OK != Tcl_GetIntFromObj(0, objv[1], &spindle)) {
		    setresult(interp,"lbv_spindle_override: malformed spindle number");
		    return TCL_ERROR;
		}
		if (TCL_OK != Tcl_GetIntFromObj(0, objv[2], &percent)) {
		    setresult(interp,"lbv_spindle_override: need percent");
		    return TCL_ERROR;
		}
		sendSpindleOverride(spindle, ((double) percent) / 100.0);
		return TCL_OK;
    }

    setresult(interp,"lbv_spindle_override: need percent");
    return TCL_ERROR;
}

static int lbv_task_plan_init(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (0 != sendTaskPlanInit()) {
	setresult(interp,"lbv_task_plan_init: can't init interpreter");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_open(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_open: need file");
	return TCL_ERROR;
    }

    if (0 != sendProgramOpen(Tcl_GetStringFromObj(objv[1], 0))) {
	setresult(interp,"lbv_open: can't open file");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_run(ClientData clientdata,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    int line;

    CHECKLBV
    if (objc == 1) {
	if (0 != sendProgramRun(0)) {
	    setresult(interp,"lbv_run: can't execute program");
	    return TCL_OK;
	}
    }

    if (objc == 2) {
	if (0 != Tcl_GetIntFromObj(0, objv[1], &line)) {
	    setresult(interp,"lbv_run: need integer start line");
	    return TCL_ERROR;
	}
	if (0 != sendProgramRun(line)) {
	    setresult(interp,"lbv_run: can't execute program");
	    return TCL_OK;
	}
    }

    return TCL_OK;
}

static int lbv_pause(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (0 != sendProgramPause()) {
	setresult(interp,"lbv_pause: can't pause program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_optional_stop(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    Tcl_Obj *obj;
    int on;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return status
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	// get the current state from the status
	obj = Tcl_NewIntObj(lbvStatus->task.optional_stop_state);
	Tcl_SetObjResult(interp, obj);
	return TCL_OK;
    }

    if (objc == 2) {
	if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &on)) {
	    if (0 != sendSetOptionalStop(on)) {
		    setresult(interp,"lbv_optional_stop: can't send command");
		    return TCL_OK;
	    }
	    return TCL_OK;
	} else {
	    setresult(interp,"lbv_optional_stop: need 0 or 1");
	    return TCL_ERROR;
	}
    }

    setresult(interp,"lbv_optional_stop: need no args, 0 or 1");
    return TCL_ERROR;
}

static int lbv_resume(ClientData clientdata,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (0 != sendProgramResume()) {
	setresult(interp,"lbv_resume: can't resume program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_step(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (0 != sendProgramStep()) {
	setresult(interp,"lbv_step: can't step program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_abort(ClientData clientdata,
		     Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (0 != sendAbort()) {
	setresult(interp,"lbv_abort: can't execute program");
	return TCL_OK;
    }

    return TCL_OK;
}

static int lbv_program(ClientData clientdata,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_program: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (0 != lbvStatus->task.file[0]) {
	setresult(interp,lbvStatus->task.file);
	return TCL_OK;
    }

    setresult(interp,"none");
    return TCL_OK;
}

static int lbv_program_status(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_program_status: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    switch (lbvStatus->task.interpState) {
    case LBV_TASK_INTERP_READING:
    case LBV_TASK_INTERP_WAITING:
	setresult(interp,"running");
	return TCL_OK;
	break;

    case LBV_TASK_INTERP_PAUSED:
	setresult(interp,"paused");
	return TCL_OK;
	break;

    default:
	setresult(interp,"idle");
	return TCL_OK;
    }

    setresult(interp,"idle");
    return TCL_OK;
}

static int lbv_program_line(ClientData clientdata,
			    Tcl_Interp * interp, int objc,
			    Tcl_Obj * CONST objv[])
{
    Tcl_Obj *lineobj;
    int programActiveLine = 0;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_program_line: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (programStartLine < 0
	|| lbvStatus->task.readLine < programStartLine) {
	// controller is skipping lines
	programActiveLine = lbvStatus->task.readLine;
    } else {			// controller is not skipping lines
	if (lbvStatus->task.currentLine > 0) {
	    if (lbvStatus->task.motionLine > 0 &&
		lbvStatus->task.motionLine < lbvStatus->task.currentLine) {
		// active line is the motion line, which lags
		programActiveLine = lbvStatus->task.motionLine;
	    } else {
		// active line is the current line-- no motion lag
		programActiveLine = lbvStatus->task.currentLine;
	    }
	} else {
	    // no active line at all
	    programActiveLine = 0;
	}
    }				// end of else controller is not skipping
    // lines

    lineobj = Tcl_NewIntObj(programActiveLine);

    Tcl_SetObjResult(interp, lineobj);
    return TCL_OK;
}

static int lbv_program_codes(ClientData clientdata,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    char codes_string[256];
    char string[256];
    int t;
    int code;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_program_codes: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }
    // fill in the active G codes
    codes_string[0] = 0;
    for (t = 1; t < ACTIVE_G_CODES; t++) {
	code = lbvStatus->task.activeGCodes[t];
	if (code == -1) {
	    continue;
	}
	if (code % 10) {
	    sprintf(string, "G%.1f ", (double) code / 10.0);
	} else {
	    sprintf(string, "G%d ", code / 10);
	}
	strcat(codes_string, string);
    }

    // fill in the active M codes, settings too
    for (t = 1; t < ACTIVE_M_CODES; t++) {
	code = lbvStatus->task.activeMCodes[t];
	if (code == -1) {
	    continue;
	}
	sprintf(string, "M%d ", code);
	strcat(codes_string, string);
    }

    // fill in F and S codes also
    sprintf(string, "F%.0f ", lbvStatus->task.activeSettings[1]);
    strcat(codes_string, string);
    sprintf(string, "S%.0f", fabs(lbvStatus->task.activeSettings[2]));
    strcat(codes_string, string);

    setresult(interp,codes_string);
    return TCL_OK;
}

static int lbv_joint_type(ClientData clientdata,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_joint_type: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	    setresult(interp,"lbv_joint_type: joint out of bounds");
	    return TCL_ERROR;
	}

	switch (lbvStatus->motion.joint[joint].jointType) {
	case LBV_LINEAR:
	    setresult(interp,"linear");
	    break;
	case LBV_ANGULAR:
	    setresult(interp,"angular");
	    break;
	default:
	    setresult(interp,"custom");
	    break;
	}

	return TCL_OK;
    }

    setresult(interp,"lbv_joint_type: invalid joint number");
    return TCL_ERROR;
}

static int lbv_joint_units(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_joint_units: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    if (TCL_OK == Tcl_GetIntFromObj(0, objv[1], &joint)) {
	if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	    setresult(interp,"lbv_joint_units: joint out of bounds");
	    return TCL_ERROR;
	}

	switch (lbvStatus->motion.joint[joint].jointType) {
	case LBV_LINEAR:
	    /* try mm */
	    if (CLOSE(lbvStatus->motion.joint[joint].units, 1.0,
		      LINEAR_CLOSENESS)) {
		setresult(interp,"mm");
		return TCL_OK;
	    }
	    /* now try inch */
	    else if (CLOSE
		     (lbvStatus->motion.joint[joint].units, INCH_PER_MM,
		      LINEAR_CLOSENESS)) {
		setresult(interp,"inch");
		return TCL_OK;
	    }
	    /* now try cm */
	    else if (CLOSE(lbvStatus->motion.joint[joint].units, CM_PER_MM,
			   LINEAR_CLOSENESS)) {
		setresult(interp,"cm");
		return TCL_OK;
	    }
	    /* else it's custom */
	    setresult(interp,"custom");
	    return TCL_OK;
	    break;

	case LBV_ANGULAR:
	    /* try degrees */
	    if (CLOSE(lbvStatus->motion.joint[joint].units, 1.0,
		      ANGULAR_CLOSENESS)) {
		setresult(interp,"deg");
		return TCL_OK;
	    }
	    /* now try radians */
	    else if (CLOSE
		     (lbvStatus->motion.joint[joint].units, RAD_PER_DEG,
		      ANGULAR_CLOSENESS)) {
		setresult(interp,"rad");
		return TCL_OK;
	    }
	    /* now try grads */
	    else if (CLOSE
		     (lbvStatus->motion.joint[joint].units, GRAD_PER_DEG,
		      ANGULAR_CLOSENESS)) {
		setresult(interp,"grad");
		return TCL_OK;
	    }
	    /* else it's custom */
	    setresult(interp,"custom");
	    return TCL_OK;
	    break;

	default:
	    setresult(interp,"custom");
	    return TCL_OK;
	    break;
	}
    }

    setresult(interp,"lbv_joint_units: invalid joint number");
    return TCL_ERROR;
}

static int lbv_program_linear_units(ClientData clientdata,
				    Tcl_Interp * interp, int objc,
				    Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_program_linear_units: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    switch (lbvStatus->task.programUnits) {
    case CANON_UNITS_INCHES:
	setresult(interp,"inch");
	return TCL_OK;
	break;

    case CANON_UNITS_MM:
	setresult(interp,"mm");
	return TCL_OK;
	break;

    case CANON_UNITS_CM:
	setresult(interp,"cm");
	return TCL_OK;
	break;

    default:
	setresult(interp,"custom");
	return TCL_OK;
	break;
    }

    setresult(interp,"custom");
    return TCL_OK;
}

static int lbv_program_angular_units(ClientData clientdata,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_program_angular_units: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }
    // currently the LBV doesn't have separate program angular units, so
    // these are simply "deg"
    setresult(interp,"deg");
    return TCL_OK;
}

static int lbv_user_linear_units(ClientData clientdata,
				 Tcl_Interp * interp, int objc,
				 Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_user_linear_units: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    /* try mm */
    if (CLOSE(lbvStatus->motion.traj.linearUnits, 1.0, LINEAR_CLOSENESS)) {
	setresult(interp,"mm");
	return TCL_OK;
    }
    /* now try inch */
    else if (CLOSE(lbvStatus->motion.traj.linearUnits, INCH_PER_MM,
		   LINEAR_CLOSENESS)) {
	setresult(interp,"inch");
	return TCL_OK;
    }
    /* now try cm */
    else if (CLOSE(lbvStatus->motion.traj.linearUnits, CM_PER_MM,
		   LINEAR_CLOSENESS)) {
	setresult(interp,"cm");
	return TCL_OK;
    }

    /* else it's custom */
    setresult(interp,"custom");
    return TCL_OK;
}

static int lbv_user_angular_units(ClientData clientdata,
				  Tcl_Interp * interp, int objc,
				  Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_user_angular_units: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    /* try degrees */
    if (CLOSE(lbvStatus->motion.traj.angularUnits, 1.0, ANGULAR_CLOSENESS)) {
	setresult(interp,"deg");
	return TCL_OK;
    }
    /* now try radians */
    else if (CLOSE(lbvStatus->motion.traj.angularUnits, RAD_PER_DEG,
		   ANGULAR_CLOSENESS)) {
	setresult(interp,"rad");
	return TCL_OK;
    }
    /* now try grads */
    else if (CLOSE(lbvStatus->motion.traj.angularUnits, GRAD_PER_DEG,
		   ANGULAR_CLOSENESS)) {
	setresult(interp,"grad");
	return TCL_OK;
    }

    /* else it's an abitrary number, so just return it */
    setresult(interp,"custom");
    return TCL_OK;
}

static int lbv_display_linear_units(ClientData clientdata,
				    Tcl_Interp * interp, int objc,
				    Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_display_linear_units: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    switch (linearUnitConversion) {
    case LINEAR_UNITS_INCH:
	setresult(interp,"inch");
	break;
    case LINEAR_UNITS_MM:
	setresult(interp,"mm");
	break;
    case LINEAR_UNITS_CM:
	setresult(interp,"cm");
	break;
    case LINEAR_UNITS_AUTO:
	switch (lbvStatus->task.programUnits) {
	case CANON_UNITS_MM:
	    setresult(interp,"(mm)");
	    break;
	case CANON_UNITS_INCHES:
	    setresult(interp,"(inch)");
	    break;
	case CANON_UNITS_CM:
	    setresult(interp,"(cm)");
	    break;
	}
	break;
    default:
	setresult(interp,"custom");
	break;
    }

    return TCL_OK;
}

static int lbv_display_angular_units(ClientData clientdata,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST objv[])
{
    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_display_angular_units: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    switch (angularUnitConversion) {
    case ANGULAR_UNITS_DEG:
	setresult(interp,"deg");
	break;
    case ANGULAR_UNITS_RAD:
	setresult(interp,"rad");
	break;
    case ANGULAR_UNITS_GRAD:
	setresult(interp,"grad");
	break;
    case ANGULAR_UNITS_AUTO:
	setresult(interp,"(deg)");	/*! \todo FIXME-- always deg? */
	break;
    default:
	setresult(interp,"custom");
	break;
    }

    return TCL_OK;
}

static int lbv_linear_unit_conversion(ClientData clientdata,
				      Tcl_Interp * interp, int objc,
				      Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return unit setting
	switch (linearUnitConversion) {
	case LINEAR_UNITS_INCH:
	    setresult(interp,"inch");
	    break;
	case LINEAR_UNITS_MM:
	    setresult(interp,"mm");
	    break;
	case LINEAR_UNITS_CM:
	    setresult(interp,"cm");
	    break;
	case LINEAR_UNITS_AUTO:
	    setresult(interp,"auto");
	    break;
	default:
	    setresult(interp,"custom");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "inch")) {
	    linearUnitConversion = LINEAR_UNITS_INCH;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "mm")) {
	    linearUnitConversion = LINEAR_UNITS_MM;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "cm")) {
	    linearUnitConversion = LINEAR_UNITS_CM;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    linearUnitConversion = LINEAR_UNITS_AUTO;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "custom")) {
	    linearUnitConversion = LINEAR_UNITS_CUSTOM;
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_linear_unit_conversion: need 'inch', 'mm', 'cm', 'auto', 'custom', or no args");
    return TCL_ERROR;
}

static int lbv_angular_unit_conversion(ClientData clientdata,
				       Tcl_Interp * interp, int objc,
				       Tcl_Obj * CONST objv[])
{
    char *objstr;

    CHECKLBV
    if (objc == 1) {
	// no arg-- return unit setting
	switch (angularUnitConversion) {
	case ANGULAR_UNITS_DEG:
	    setresult(interp,"deg");
	    break;
	case ANGULAR_UNITS_RAD:
	    setresult(interp,"rad");
	    break;
	case ANGULAR_UNITS_GRAD:
	    setresult(interp,"grad");
	    break;
	case ANGULAR_UNITS_AUTO:
	    setresult(interp,"auto");
	    break;
	default:
	    setresult(interp,"custom");
	    break;
	}
	return TCL_OK;
    }

    if (objc == 2) {
	objstr = Tcl_GetStringFromObj(objv[1], 0);
	if (!strcmp(objstr, "deg")) {
	    angularUnitConversion = ANGULAR_UNITS_DEG;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "rad")) {
	    angularUnitConversion = ANGULAR_UNITS_RAD;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "grad")) {
	    angularUnitConversion = ANGULAR_UNITS_GRAD;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "auto")) {
	    angularUnitConversion = ANGULAR_UNITS_AUTO;
	    return TCL_OK;
	}
	if (!strcmp(objstr, "custom")) {
	    angularUnitConversion = ANGULAR_UNITS_CUSTOM;
	    return TCL_OK;
	}
    }

    setresult(interp,"lbv_angular_unit_conversion: need 'deg', 'rad', 'grad', 'auto', 'custom', or no args");
    return TCL_ERROR;
}

static int lbv_task_heartbeat(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    Tcl_Obj *hbobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_task_heartbeat: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    hbobj = Tcl_NewIntObj(lbvStatus->task.heartbeat);

    Tcl_SetObjResult(interp, hbobj);
    return TCL_OK;
}

static int lbv_task_command(ClientData clientdata,
			    Tcl_Interp * interp, int objc,
			    Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_task_command: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandobj = Tcl_NewIntObj(lbvStatus->task.command_type);

    Tcl_SetObjResult(interp, commandobj);
    return TCL_OK;
}

static int lbv_task_command_number(ClientData clientdata,
				   Tcl_Interp * interp, int objc,
				   Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandnumber;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_task_command_number: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandnumber = Tcl_NewIntObj(lbvStatus->task.echo_serial_number);

    Tcl_SetObjResult(interp, commandnumber);
    return TCL_OK;
}

static int lbv_task_command_status(ClientData clientdata,
				   Tcl_Interp * interp, int objc,
				   Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandstatus;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_task_command_status: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandstatus = Tcl_NewIntObj(lbvStatus->task.status);

    Tcl_SetObjResult(interp, commandstatus);
    return TCL_OK;
}

static int lbv_io_heartbeat(ClientData clientdata,
			    Tcl_Interp * interp, int objc,
			    Tcl_Obj * CONST objv[])
{
    Tcl_Obj *hbobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_io_heartbeat: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    hbobj = Tcl_NewIntObj(lbvStatus->io.heartbeat);

    Tcl_SetObjResult(interp, hbobj);
    return TCL_OK;
}

static int lbv_io_command(ClientData clientdata,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_io_command: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandobj = Tcl_NewIntObj(lbvStatus->io.command_type);

    Tcl_SetObjResult(interp, commandobj);
    return TCL_OK;
}

static int lbv_io_command_number(ClientData clientdata,
				 Tcl_Interp * interp, int objc,
				 Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandnumber;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_io_command_number: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandnumber = Tcl_NewIntObj(lbvStatus->io.echo_serial_number);

    Tcl_SetObjResult(interp, commandnumber);
    return TCL_OK;
}

static int lbv_io_command_status(ClientData clientdata,
				 Tcl_Interp * interp, int objc,
				 Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandstatus;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_io_command_status: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandstatus = Tcl_NewIntObj(lbvStatus->io.status);

    Tcl_SetObjResult(interp, commandstatus);
    return TCL_OK;
}

static int lbv_motion_heartbeat(ClientData clientdata,
				Tcl_Interp * interp, int objc,
				Tcl_Obj * CONST objv[])
{
    Tcl_Obj *hbobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_motion_heartbeat: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    hbobj = Tcl_NewIntObj(lbvStatus->motion.heartbeat);

    Tcl_SetObjResult(interp, hbobj);
    return TCL_OK;
}

static int lbv_motion_command(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandobj;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_motion_command: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandobj = Tcl_NewIntObj(lbvStatus->motion.command_type);

    Tcl_SetObjResult(interp, commandobj);
    return TCL_OK;
}

static int lbv_motion_command_number(ClientData clientdata,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandnumber;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_motion_command_number: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandnumber = Tcl_NewIntObj(lbvStatus->motion.echo_serial_number);

    Tcl_SetObjResult(interp, commandnumber);
    return TCL_OK;
}

static int lbv_motion_command_status(ClientData clientdata,
				     Tcl_Interp * interp, int objc,
				     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *commandstatus;

    CHECKLBV
    if (objc != 1) {
	setresult(interp,"lbv_motion_command_status: need no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    commandstatus = Tcl_NewIntObj(lbvStatus->motion.status);

    Tcl_SetObjResult(interp, commandstatus);
    return TCL_OK;
}

static int lbv_joint_backlash(ClientData clientdata,
			     Tcl_Interp * interp, int objc,
			     Tcl_Obj * CONST objv[])
{
    Tcl_Obj *valobj;
    int joint;
    double backlash;

    // syntax is lbv_joint_backlash <joint> {<backlash>}
    // if <backlash> is not specified, returns current value,
    // otherwise, sets backlash to specified value

    CHECKLBV
    // check number of args supplied
    if ((objc < 2) || (objc > 3)) {
	setresult(interp,"lbv_joint_backlash: need <joint> {<backlash>}");
	return TCL_ERROR;
    }
    // get joint number
    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint) ||
	joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	setresult(interp,"lbv_joint_backlash: need joint as integer, 0..LBVMOT_MAX_JOINTS-1");
	return TCL_ERROR;
    }
    // test for get or set
    if (objc == 2) {
	// want to get present value
	valobj = Tcl_NewDoubleObj(lbvStatus->motion.joint[joint].backlash);
	Tcl_SetObjResult(interp, valobj);
	return TCL_OK;
    } else {
	// want to set new value
	if (0 != Tcl_GetDoubleFromObj(0, objv[2], &backlash)) {
	    setresult(interp,"lbv_joint_backlash: need backlash as real number");
	    return TCL_ERROR;
	}
	// write it out
	sendJointSetBacklash(joint, backlash);
	return TCL_OK;
    }
}

static int lbv_joint_enable(ClientData clientdata,
			   Tcl_Interp * interp, int objc,
			   Tcl_Obj * CONST objv[])
{
    int joint;
    int val;
    Tcl_Obj *enobj;

    // syntax is lbv_joint_output <joint> {0 | 1}

    CHECKLBV
    if (objc < 2) {
	setresult(interp,"lbv_joint_enable: need <joint>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint) ||
	joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	setresult(interp,"lbv_joint_enable: need joint as integer, 0..LBVMOT_MAX_JOINTS-1");
	return TCL_ERROR;
    }

    if (objc == 2) {
	if (lbvUpdateType == LBV_UPDATE_AUTO) {
	    updateStatus();
	}
	enobj = Tcl_NewIntObj(lbvStatus->motion.joint[joint].enabled);
	Tcl_SetObjResult(interp, enobj);
	return TCL_OK;
    }
    // else we were given 0 or 1 to enable/disable it
    if (0 != Tcl_GetIntFromObj(0, objv[2], &val)) {
	setresult(interp,"lbv_joint_enable: need 0, 1 for disable, enable");
	return TCL_ERROR;
    }

    sendJointEnable(joint, val);
    return TCL_OK;
}

static int lbv_joint_load_comp(ClientData clientdata,
			      Tcl_Interp * interp, int objc,
			      Tcl_Obj * CONST objv[])
{
    int joint, type;
    char file[256];

    CHECKLBV
    // syntax is lbv_joint_load_comp <joint> <file>

    if (objc != 4) {
	setresult(interp,"lbv_joint_load_comp: need <joint> <file> <type>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetIntFromObj(0, objv[1], &joint) ||
	joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	setresult(interp,"lbv_joint_load_comp: need joint as integer, 0..LBVMOT_MAX_JOINTS-1");
	return TCL_ERROR;
    }
    // copy objv[1] to file arg, to make sure it's not modified
    strcpy(file, Tcl_GetStringFromObj(objv[2], 0));

    if (0 != Tcl_GetIntFromObj(0, objv[3], &type)) {
	setresult(interp,"lbv_joint_load_comp: <type> must be an int");
    }

    // now write it out
    sendJointLoadComp(joint, file, type);
    return TCL_OK;
}

int lbv_teleop_enable(ClientData clientdata,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    int enable;

    if (objc != 1) {
	if (0 != Tcl_GetIntFromObj(0, objv[1], &enable)) {
	    setresult(interp,"lbv_teleop_enable: <enable> must be an integer");
	    return TCL_ERROR;
	}
	sendSetTeleopEnable(enable);
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(lbvStatus->motion.traj.mode ==
				   LBV_TRAJ_MODE_TELEOP));
    return TCL_OK;
}

int lbv_kinematics_type(ClientData clientdata,
			Tcl_Interp * interp, int objc,
			Tcl_Obj * CONST objv[])
{

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(lbvStatus->motion.traj.
				   kinematics_type));
    return TCL_OK;
}

int lbv_probe_clear(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    if (objc != 1) {
	setresult(interp,"lbv_probe_clear: needs no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp, Tcl_NewIntObj(sendClearProbeTrippedFlag()));
    return TCL_OK;
}

int lbv_probe_value(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    if (objc != 1) {
	setresult(interp,"lbv_probe_value: needs no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(lbvStatus->motion.traj.probeval));
    return TCL_OK;
}

int lbv_probe_tripped(ClientData clientdata,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    if (objc != 1) {
	setresult(interp,"lbv_probe_tripped: needs no args");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(lbvStatus->motion.traj.probe_tripped));
    return TCL_OK;
}

int lbv_probe_move(ClientData clientdata,
		   Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    double x, y, z;

    if (objc != 4) {
	setresult(interp,"lbv_probe_move: <x> <y> <z>");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetDoubleFromObj(0, objv[1], &x)) {
	setresult(interp,"lbv_probe_move: <x> must be a double");
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[2], &y)) {
	setresult(interp,"lbv_probe_move: <y> must be a double");
    }
    if (0 != Tcl_GetDoubleFromObj(0, objv[3], &z)) {
	setresult(interp,"lbv_probe_move: <z> must be a double");
    }

    Tcl_SetObjResult(interp, Tcl_NewIntObj(sendProbe(x, y, z)));
    return TCL_OK;
}

static int lbv_probed_pos(ClientData clientdata,
			  Tcl_Interp * interp, int objc,
			  Tcl_Obj * CONST objv[])
{
    char string[1];
    Tcl_Obj *posobj;

    CHECKLBV
    if (objc != 2) {
	setresult(interp,"lbv_probed_pos: need exactly 1 non-negative integer");
	return TCL_ERROR;
    }

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }

    strncpy(string, Tcl_GetStringFromObj(objv[1], 0),1);

    switch (string[0]) {
    case 'x': case 'X':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.probedPosition.tran.x));
        break;
    case 'y': case 'Y':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.probedPosition.tran.y));
        break;
    case 'z': case 'Z':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.probedPosition.tran.z));
        break;
    case 'a': case 'A':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.probedPosition.a));
        break;
    case 'b': case 'B':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.probedPosition.b));
        break;
    case 'c': case 'C':
        posobj = Tcl_NewDoubleObj(convertAngularUnits(
                                  lbvStatus->motion.traj.probedPosition.c));
        break;
    case 'u': case 'U':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.probedPosition.u));
        break;
    case 'v': case 'V':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.probedPosition.v));
        break;
    case 'w': case 'W':
        posobj = Tcl_NewDoubleObj(convertLinearUnits(
                                  lbvStatus->motion.traj.probedPosition.w));
        break;
    default:
        setresult(interp,"lbv_probed_pos: bad coordinate letter argument");
        return TCL_ERROR;
    }
    Tcl_SetObjResult(interp, posobj);
    return TCL_OK;
}

// ********************************************************************
//      Pendant read routine from /dev/psaux, /dev/ttyS0, or /dev/ttyS1
// *********************************************************************

static int lbv_pendant(ClientData clientdata,
		       Tcl_Interp * interp, int objc,
		       Tcl_Obj * CONST objv[])
{
    FILE *inFile;

    char inBytes[5];
    const char *port;

    inBytes[0] = 0;
    inBytes[1] = 0;
    inBytes[2] = 0;
    inBytes[3] = 0;
    inBytes[4] = 0;

    CHECKLBV
    if (objc == 2) {
	port = Tcl_GetStringFromObj(objv[1], 0);
	if ((!strcmp(port, "/dev/psaux")) | (!strcmp(port,
						     "/dev/ttyS0")) |
	    (!strcmp(port, "/dev/ttyS1"))) {
	    inFile = fopen(port, "r+b");

	    if (inFile) {
		if (strcmp(port, "/dev/psaux")) {	// For Serial mice
		    inBytes[1] = fgetc(inFile);	// read the first Byte
		    if (inBytes[1] != 77) {	// If first byte not "M"
			fputc(77, inFile);	// Request data resent
			fflush(inFile);
			inBytes[1] = fgetc(inFile);	// and hope it is
			// correct
		    }
		}
		inBytes[4] = fgetc(inFile);	// Status byte
		inBytes[2] = fgetc(inFile);	// Horizontal movement
		inBytes[3] = fgetc(inFile);	// Vertical Movement
	    }
	    fclose(inFile);

	    if (!strcmp(port, "/dev/psaux")) {	// For PS/2
		inBytes[0] = (inBytes[4] & 0x01);	// Left button
		inBytes[1] = (inBytes[4] & 0x02) >> 1;	// Right button
	    } else {		// For serial mice
		inBytes[0] = (inBytes[4] & 0x20) >> 5;	// Left button
		inBytes[1] = (inBytes[4] & 0x10) >> 4;	// Right button
		if (inBytes[4] & 0x02) {
		    inBytes[2] = inBytes[2] | 0xc0;
		}
		if (inBytes[4] & 0x08) {
		    inBytes[3] = inBytes[3] | 0xc0;
		}
	    }

	    char buf[80];
	    snprintf(buf, sizeof(buf), "%i %i %d %d %i", inBytes[0],
		    inBytes[1], inBytes[2], inBytes[3], inBytes[4]);
	    Tcl_SetResult(interp, buf, TCL_VOLATILE);
	    return TCL_OK;
	}
    }
    setresult(interp,"Need /dev/psaux, /dev/ttyS0 or /dev/ttyS1 as Arg");
    return TCL_ERROR;
}

// *******************************************************************

// provide some of the extended Tcl builtins not available for various plats
// "int", as in "int 3.9" which returns 3
static int localint(ClientData clientdata,
		    Tcl_Interp * interp, int objc, Tcl_Obj * CONST objv[])
{
    double val;
    char resstring[80];

    if (objc != 2) {
	// need exactly one arg
	setresult(interp, "wrong # args: should be \"int value\"");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetDoubleFromObj(0, objv[1], &val)) {
	resstring[0] = 0;
	strcat(resstring, "expected number but got \"");
	strncat(resstring, Tcl_GetStringFromObj(objv[1], 0),
		sizeof(resstring) - strlen(resstring) - 2);
	strcat(resstring, "\"");
	setresult(interp, resstring);
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp, Tcl_NewIntObj((int) val));
    return TCL_OK;
}

static const char *one_head(int x0, int y0, int x1, int y1)
{
    static char buf[100];
    snprintf(buf, sizeof(buf), "%d %d %d %d", x0, y0, x1, y1);
    return buf;
}

// "round", as in "round 3.9" which returns 4
static int localround(ClientData clientdata,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    double val;
    char resstring[80];

    if (objc != 2) {
	// need exactly one arg
	setresult(interp,"wrong # args: should be \"round value\"");
	return TCL_ERROR;
    }

    if (0 != Tcl_GetDoubleFromObj(0, objv[1], &val)) {
	resstring[0] = 0;
	strcat(resstring, "expected number but got \"");
	strncat(resstring, Tcl_GetStringFromObj(objv[1], 0),
		sizeof(resstring) - strlen(resstring) - 2);
	strcat(resstring, "\"");
	setresult(interp,resstring);
	return TCL_ERROR;
    }

    Tcl_SetObjResult(interp,
		     Tcl_NewIntObj(val <
				   0.0 ? (int) (val - 0.5) : (int) (val +
								    0.5)));
    return TCL_OK;
}

#include <X11/extensions/Xinerama.h>

static int multihead(ClientData clientdata,
		      Tcl_Interp * interp, int objc,
		      Tcl_Obj * CONST objv[])
{
    if(objc > 1)
	setresult(interp,"wrong # args: should be \"multihead\"");

    Tk_Window tkwin = Tk_MainWindow(interp);
    if(!tkwin) return TCL_ERROR;

    Display *d = Tk_Display(tkwin);
    if(!d) return TCL_ERROR;

    Tcl_ResetResult(interp);

    XineramaScreenInfo *inf = NULL;
    int count = 0;

    int i, j;
    if(XineramaQueryExtension(d, &i, &j)) {
        inf = XineramaQueryScreens(d, &count);
    }

    if( !inf ) {
        Tcl_AppendElement(interp, one_head(0, 0,
                   DisplayWidth(d, DefaultScreen(d)),
                   DisplayHeight(d, DefaultScreen(d))));
    } else {
        for(i=0; i<count; i++) {
            Tcl_AppendElement(interp, one_head(inf[i].x_org, inf[i].y_org,
                        inf[i].x_org + inf[i].width,
                        inf[i].y_org + inf[i].height));
        }
        XFree(inf);
    }
    return TCL_OK;
}

static void sigQuit(int sig)
{
    thisQuit((ClientData) 0);
}

static void initMain()
{
    lbvWaitType = LBV_WAIT_RECEIVED;
    lbvCommandSerialNumber = 0;
    lbvTimeout = 0.0;
    lbvUpdateType = LBV_UPDATE_AUTO;
    linearUnitConversion = LINEAR_UNITS_AUTO;
    angularUnitConversion = ANGULAR_UNITS_AUTO;
    lbvCommandBuffer = 0;
    lbvStatusBuffer = 0;
    lbvStatus = 0;

    lbvErrorBuffer = 0;
    error_string[LINELEN-1] = 0;
    operator_text_string[LINELEN-1] = 0;
    operator_display_string[LINELEN-1] = 0;
    programStartLine = 0;
}

int lbv_init(ClientData cd, Tcl_Interp *interp, int argc, const char **argv)
{
    bool quick = false;
    initMain();
    // process command line args
    // use -ini inifilename to set LBV_INIFILE
    // see lbvargs.c for other arguments
    // use -quick to return quickly if lbv is not running
    if (0 != lbvGetArgs(argc, (char**)argv)) {
        setresult(interp,"error in argument list\n");
        return TCL_ERROR;
    }
    // get configuration information
    iniLoad(lbv_inifile);

    for(int i=1; i<argc; i++)
    {
	if(!strcmp(argv[i], "-quick")) quick = true;
    }

    // update tcl's idea of the inifile name
    Tcl_SetVar(interp, "LBV_INIFILE", lbv_inifile, TCL_GLOBAL_ONLY);

    // init NML
    if (0 != tryNml(quick ? 0.0 : 10.0, quick ? 0.0 : 1.0)) {
        setresult(interp,"no lbv connection");
        thisQuit(NULL);
        return TCL_ERROR;
    }
    // get current serial number, and save it for restoring when we quit
    // so as not to interfere with real operator interface
    updateStatus();
    lbvCommandSerialNumber = lbvStatus->echo_serial_number;

    // attach our quit function to exit
    Tcl_CreateExitHandler(thisQuit, (ClientData) 0);

    // attach our quit function to SIGINT
    signal(SIGINT, sigQuit);

    setresult(interp,"");
    return TCL_OK;
}

extern "C" 
int Labvcnc_Init(Tcl_Interp * interp);
int Labvcnc_Init(Tcl_Interp * interp)
{
    if (Tcl_InitStubs(interp, "8.1", 0) == NULL) 
    {
        return TCL_ERROR;
    }

    /* 
     * Call Tcl_CreateCommand for application-specific commands, if
     * they weren't already created by the init procedures called above.
     */

    Tcl_CreateCommand(interp, "lbv_init", lbv_init, (ClientData) NULL,
                         (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_plat", lbv_plat, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_ini", lbv_ini, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_debug", lbv_Debug, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_set_wait", lbv_set_wait,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_wait", lbv_wait, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_set_timeout", lbv_set_timeout,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_update", lbv_update,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_time", lbv_time, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_error", lbv_error, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_operator_text", lbv_operator_text,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_operator_display",
			 lbv_operator_display, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_estop", lbv_estop, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_machine", lbv_machine,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_mode", lbv_mode, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_mist", lbv_mist, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_flood", lbv_flood, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_lube", lbv_lube, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_lube_level", lbv_lube_level,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_spindle", lbv_spindle,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_brake", lbv_brake, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_tool", lbv_tool, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_tool_offset", lbv_tool_offset,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_load_tool_table",
			 lbv_load_tool_table, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_set_tool_offset",
			 lbv_set_tool_offset, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_abs_cmd_pos", lbv_abs_cmd_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_abs_act_pos", lbv_abs_act_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_rel_cmd_pos", lbv_rel_cmd_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_rel_act_pos", lbv_rel_act_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_pos", lbv_joint_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_pos_offset", lbv_pos_offset,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_limit", lbv_joint_limit,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_fault", lbv_joint_fault,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_override_limit", lbv_override_limit,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_optional_stop", lbv_optional_stop,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_homed", lbv_joint_homed,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_mdi", lbv_mdi, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_home", lbv_home, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_unhome", lbv_unhome, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_jog_stop", lbv_jog_stop,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_jog", lbv_jog, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_jog_incr", lbv_jog_incr,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_feed_override", lbv_feed_override,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_rapid_override", lbv_rapid_override,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_spindle_override", lbv_spindle_override,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_task_plan_init", lbv_task_plan_init,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_open", lbv_open, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_run", lbv_run, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_pause", lbv_pause, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_resume", lbv_resume,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_step", lbv_step, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_abort", lbv_abort, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_program", lbv_program,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_program_line", lbv_program_line,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_program_status", lbv_program_status,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_program_codes", lbv_program_codes,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_type", lbv_joint_type,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_units", lbv_joint_units,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_program_linear_units",
			 lbv_program_linear_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_program_angular_units",
			 lbv_program_angular_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_user_linear_units",
			 lbv_user_linear_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_user_angular_units",
			 lbv_user_angular_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_display_linear_units",
			 lbv_display_linear_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_display_angular_units",
			 lbv_display_angular_units, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_linear_unit_conversion",
			 lbv_linear_unit_conversion, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_angular_unit_conversion",
			 lbv_angular_unit_conversion, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_task_heartbeat", lbv_task_heartbeat,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_task_command", lbv_task_command,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_task_command_number",
			 lbv_task_command_number, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_task_command_status",
			 lbv_task_command_status, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_io_heartbeat", lbv_io_heartbeat,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_io_command", lbv_io_command,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_io_command_number",
			 lbv_io_command_number, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_io_command_status",
			 lbv_io_command_status, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_motion_heartbeat",
			 lbv_motion_heartbeat, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_motion_command", lbv_motion_command,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_motion_command_number",
			 lbv_motion_command_number, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_motion_command_status",
			 lbv_motion_command_status, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_backlash", lbv_joint_backlash,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_load_comp", lbv_joint_load_comp,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_joint_enable", lbv_joint_enable,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_teleop_enable", lbv_teleop_enable,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_kinematics_type",
			 lbv_kinematics_type, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_probe_clear", lbv_probe_clear,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "lbv_probe_value", lbv_probe_value,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "lbv_probe_tripped", lbv_probe_tripped,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "lbv_probe_move", lbv_probe_move,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);
    Tcl_CreateObjCommand(interp, "lbv_probed_pos", lbv_probed_pos,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "lbv_pendant", lbv_pendant,
			 (ClientData) NULL, (Tcl_CmdDeleteProc *) NULL);

    // provide builtins that may have been left out

    Tcl_CreateObjCommand(interp, "int", localint, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "round", localround, (ClientData) NULL,
			 (Tcl_CmdDeleteProc *) NULL);

    Tcl_CreateObjCommand(interp, "multihead", multihead, (ClientData) NULL,
                         (Tcl_CmdDeleteProc*) NULL);

    /* 
     * Specify a user-specific startup file to invoke if the application
     * is run interactively.  Typically the startup file is "~/.apprc"
     * where "app" is the name of the application.  If this line is deleted
     * then no user-specific startup file will be run under any conditions.
     */

    Tcl_SetVar(interp, "tcl_rcFileName", "~/.lbvshrc", TCL_GLOBAL_ONLY);

    // set app-specific global variables
    Tcl_SetVar(interp, "LBV_INIFILE", lbv_inifile, TCL_GLOBAL_ONLY);
    Tcl_PkgProvide(interp, "Labvcnc", "1.0");

    Tcl_ResetResult(interp);

    return TCL_OK;
}

