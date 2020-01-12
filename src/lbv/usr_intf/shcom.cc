/********************************************************************
* Description: shcom.cc
*   Common functions for NML calls
*
*   Derived from a work by Fred Proctor & Will Shackleford
*   Further derived from work by jmkasunich, Alex Joni
*
* Author: Eric H. Johnson
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2006 All rights reserved.
*
* Last change:
********************************************************************/


#define __STDC_FORMAT_MACROS
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <ctype.h>
#include <math.h>
#include <sys/types.h>
#include <inttypes.h>

#include "rcs.hh"
#include "posemath.h"		// PM_POSE, TO_RAD
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"
#include "canon.hh"		// CANON_UNITS, CANON_UNITS_INCHES,MM,CM
#include "lbvglb.h"		// LBV_NMLFILE, TRAJ_MAX_VELOCITY, etc.
#include "lbvcfg.h"		// DEFAULT_TRAJ_MAX_VELOCITY
#include "inifile.hh"		// INIFILE
#include "nml_oi.hh"            // nmlErrorFormat, NML_ERROR, etc
#include "rcs_print.hh"
#include "timer.hh"             // esleep
#include "shcom.hh"             // Common NML communications functions

LINEAR_UNIT_CONVERSION linearUnitConversion;
ANGULAR_UNIT_CONVERSION angularUnitConversion;

static int num_joints = LBVMOT_MAX_JOINTS;

int lbvCommandSerialNumber;

// the NML channels to the LBV task
RCS_CMD_CHANNEL *lbvCommandBuffer;
RCS_STAT_CHANNEL *lbvStatusBuffer;
LBV_STAT *lbvStatus;

// the NML channel for errors
NML *lbvErrorBuffer;
char error_string[NML_ERROR_LEN];
char operator_text_string[NML_TEXT_LEN];
char operator_display_string[NML_DISPLAY_LEN];
char defaultPath[80] = DEFAULT_PATH;
// default value for timeout, 0 means wait forever
double lbvTimeout;
int programStartLine;

LBV_UPDATE_TYPE lbvUpdateType;
LBV_WAIT_TYPE lbvWaitType;

void strupr(char *s)
{  
  int i;
  
  for (i = 0; i < (int)strlen(s); i++)
    if (s[i] > 96 && s[i] <= 'z')
      s[i] -= 32;
}

int lbvTaskNmlGet()
{
    int retval = 0;

    // try to connect to LBV cmd
    if (lbvCommandBuffer == 0) {
	lbvCommandBuffer =
	    new RCS_CMD_CHANNEL(lbvFormat, "lbvCommand", "xlbv",
				lbv_nmlfile);
	if (!lbvCommandBuffer->valid()) {
	    delete lbvCommandBuffer;
	    lbvCommandBuffer = 0;
	    retval = -1;
	}
    }
    // try to connect to LBV status
    if (lbvStatusBuffer == 0) {
	lbvStatusBuffer =
	    new RCS_STAT_CHANNEL(lbvFormat, "lbvStatus", "xlbv",
				 lbv_nmlfile);
	if (!lbvStatusBuffer->valid()
	    || LBV_STAT_TYPE != lbvStatusBuffer->peek()) {
	    delete lbvStatusBuffer;
	    lbvStatusBuffer = 0;
	    lbvStatus = 0;
	    retval = -1;
	} else {
	    lbvStatus = (LBV_STAT *) lbvStatusBuffer->get_address();
	}
    }

    return retval;
}

int lbvErrorNmlGet()
{
    int retval = 0;

    if (lbvErrorBuffer == 0) {
	lbvErrorBuffer =
	    new NML(nmlErrorFormat, "lbvError", "xlbv", lbv_nmlfile);
	if (!lbvErrorBuffer->valid()) {
	    delete lbvErrorBuffer;
	    lbvErrorBuffer = 0;
	    retval = -1;
	}
    }

    return retval;
}

int tryNml(double retry_time, double retry_interval)
{
    double end;
    int good;

    if ((lbv_debug & LBV_DEBUG_NML) == 0) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = retry_time;
    good = 0;
    do {
	if (0 == lbvTaskNmlGet()) {
	    good = 1;
	    break;
	}
	esleep(retry_interval);
	end -= retry_interval;
    } while (end > 0.0);
    if ((lbv_debug & LBV_DEBUG_NML) == 0) {
	set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// inhibit diag
	// messages
    }
    if (!good) {
	return -1;
    }

    if ((lbv_debug & LBV_DEBUG_NML) == 0) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = retry_time;
    good = 0;
    do {
	if (0 == lbvErrorNmlGet()) {
	    good = 1;
	    break;
	}
	esleep(retry_interval);
	end -= retry_interval;
    } while (end > 0.0);
    if ((lbv_debug & LBV_DEBUG_NML) == 0) {
	set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// inhibit diag
	// messages
    }
    if (!good) {
	return -1;
    }

    return 0;
}

int updateStatus()
{
    NMLTYPE type;

    if (0 == lbvStatus || 0 == lbvStatusBuffer
	|| !lbvStatusBuffer->valid()) {
	return -1;
    }

    switch (type = lbvStatusBuffer->peek()) {
    case -1:
	// error on CMS channel
	return -1;
	break;

    case 0:			// no new data
    case LBV_STAT_TYPE:	// new data
	// new data
	break;

    default:
	return -1;
	break;
    }

    return 0;
}

/*
  updateError() updates "errors," which are true errors and also
  operator display and text messages.
*/
int updateError()
{
    NMLTYPE type;

    if (0 == lbvErrorBuffer || !lbvErrorBuffer->valid()) {
	return -1;
    }

    switch (type = lbvErrorBuffer->read()) {
    case -1:
	// error reading channel
	return -1;
	break;

    case 0:
	// nothing new
	break;

    case LBV_OPERATOR_ERROR_TYPE:
	strncpy(error_string,
		((LBV_OPERATOR_ERROR *) (lbvErrorBuffer->get_address()))->
		error, LINELEN - 1);
	error_string[NML_ERROR_LEN - 1] = 0;
	break;

    case LBV_OPERATOR_TEXT_TYPE:
	strncpy(operator_text_string,
		((LBV_OPERATOR_TEXT *) (lbvErrorBuffer->get_address()))->
		text, LINELEN - 1);
	operator_text_string[NML_TEXT_LEN - 1] = 0;
	break;

    case LBV_OPERATOR_DISPLAY_TYPE:
	strncpy(operator_display_string,
		((LBV_OPERATOR_DISPLAY *) (lbvErrorBuffer->
					   get_address()))->display,
		LINELEN - 1);
	operator_display_string[NML_DISPLAY_LEN - 1] = 0;
	break;

    case NML_ERROR_TYPE:
	strncpy(error_string,
		((NML_ERROR *) (lbvErrorBuffer->get_address()))->error,
		NML_ERROR_LEN - 1);
	error_string[NML_ERROR_LEN - 1] = 0;
	break;

    case NML_TEXT_TYPE:
	strncpy(operator_text_string,
		((NML_TEXT *) (lbvErrorBuffer->get_address()))->text,
		NML_TEXT_LEN - 1);
	operator_text_string[NML_TEXT_LEN - 1] = 0;
	break;

    case NML_DISPLAY_TYPE:
	strncpy(operator_display_string,
		((NML_DISPLAY *) (lbvErrorBuffer->get_address()))->display,
		NML_DISPLAY_LEN - 1);
	operator_display_string[NML_DISPLAY_LEN - 1] = 0;
	break;

    default:
	// if not recognized, set the error string
	sprintf(error_string, "unrecognized error %" PRId32, type);
	return -1;
	break;
    }

    return 0;
}

#define LBV_COMMAND_DELAY   0.1	// how long to sleep between checks

int lbvCommandWaitDone()
{
    double end;
    for (end = 0.0; lbvTimeout <= 0.0 || end < lbvTimeout; end += LBV_COMMAND_DELAY) {
	updateStatus();
	int serial_diff = lbvStatus->echo_serial_number - lbvCommandSerialNumber;
	if (serial_diff < 0) {
	    continue;
	}

	if (serial_diff > 0) {
	    return 0;
	}

	if (lbvStatus->status == RCS_DONE) {
	    return 0;
	}

	if (lbvStatus->status == RCS_ERROR) {
	    return -1;
	}

	esleep(LBV_COMMAND_DELAY);
    }

    return -1;
}

int lbvCommandWaitReceived()
{
    double end;
    for (end = 0.0; lbvTimeout <= 0.0 || end < lbvTimeout; end += LBV_COMMAND_DELAY) {
	updateStatus();

	int serial_diff = lbvStatus->echo_serial_number - lbvCommandSerialNumber;
	if (serial_diff >= 0) {
	    return 0;
	}

	esleep(LBV_COMMAND_DELAY);
    }

    return -1;
}

int lbvCommandSend(RCS_CMD_MSG & cmd)
{
    // write command
    if (lbvCommandBuffer->write(&cmd)) {
        return -1;
    }
    lbvCommandSerialNumber = cmd.serial_number;
    return 0;
}


/*
  Unit conversion

  Length and angle units in the LBV status buffer are in user units, as
  defined in the INI file in [TRAJ] LINEAR,ANGULAR_UNITS. These may differ
  from the program units, and when they are the display is confusing.

  It may be desirable to synchronize the display units with the program
  units automatically, and also to break this sync and allow independent
  display of position values.

  The global variable "linearUnitConversion" is set by the Tcl commands
  lbv_linear_unit_conversion to correspond to either "inch",
  "mm", "cm", "auto", or "custom". This forces numbers to be returned in the
  units specified, in program units when "auto" is set, or not converted
  at all if "custom" is specified.

  Ditto for "angularUnitConversion", set by lbv_angular_unit_conversion
  to "deg", "rad", "grad", "auto", or "custom".

  With no args, lbv_linear/angular_unit_conversion return the setting.

  The functions convertLinearUnits and convertAngularUnits take a length
  or angle value, typically from the lbvStatus structure, and convert it
  as indicated by linearUnitConversion and angularUnitConversion, resp.
*/


/*
  to convert linear units, values are converted to mm, then to desired
  units
*/
double convertLinearUnits(double u)
{
    double in_mm;

    /* convert u to mm */
    in_mm = u / lbvStatus->motion.traj.linearUnits;

    /* convert u to display units */
    switch (linearUnitConversion) {
    case LINEAR_UNITS_MM:
	return in_mm;
	break;
    case LINEAR_UNITS_INCH:
	return in_mm * INCH_PER_MM;
	break;
    case LINEAR_UNITS_CM:
	return in_mm * CM_PER_MM;
	break;
    case LINEAR_UNITS_AUTO:
	switch (lbvStatus->task.programUnits) {
	case CANON_UNITS_MM:
	    return in_mm;
	    break;
	case CANON_UNITS_INCHES:
	    return in_mm * INCH_PER_MM;
	    break;
	case CANON_UNITS_CM:
	    return in_mm * CM_PER_MM;
	    break;
	}
	break;

    case LINEAR_UNITS_CUSTOM:
	return u;
	break;
    }

    // If it ever gets here we have an error.

    return u;
}

double convertAngularUnits(double u)
{
    // Angular units are always degrees
    return u;
}

// polarities for joint jogging, from ini file
static int jogPol[LBVMOT_MAX_JOINTS];

int sendDebug(int level)
{
    LBV_SET_DEBUG debug_msg;

    debug_msg.debug = level;
    lbvCommandSend(debug_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendEstop()
{
    LBV_TASK_SET_STATE state_msg;

    state_msg.state = LBV_TASK_STATE_ESTOP;
    lbvCommandSend(state_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendEstopReset()
{
    LBV_TASK_SET_STATE state_msg;

    state_msg.state = LBV_TASK_STATE_ESTOP_RESET;
    lbvCommandSend(state_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendMachineOn()
{
    LBV_TASK_SET_STATE state_msg;

    state_msg.state = LBV_TASK_STATE_ON;
    lbvCommandSend(state_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendMachineOff()
{
    LBV_TASK_SET_STATE state_msg;

    state_msg.state = LBV_TASK_STATE_OFF;
    lbvCommandSend(state_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendManual()
{
    LBV_TASK_SET_MODE mode_msg;

    mode_msg.mode = LBV_TASK_MODE_MANUAL;
    lbvCommandSend(mode_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendAuto()
{
    LBV_TASK_SET_MODE mode_msg;

    mode_msg.mode = LBV_TASK_MODE_AUTO;
    lbvCommandSend(mode_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendMdi()
{
    LBV_TASK_SET_MODE mode_msg;

    mode_msg.mode = LBV_TASK_MODE_MDI;
    lbvCommandSend(mode_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendOverrideLimits(int joint)
{
    LBV_JOINT_OVERRIDE_LIMITS lim_msg;

    lim_msg.joint = joint;	// neg means off, else on for all
    lbvCommandSend(lim_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendJogStop(int ja, int jjogmode)
{
    LBV_JOG_STOP lbv_jog_stop_msg;

    if (   (   (jjogmode == JOGJOINT)
            && (lbvStatus->motion.traj.mode == LBV_TRAJ_MODE_TELEOP) )
        || (   (jjogmode == JOGTELEOP )
            && (lbvStatus->motion.traj.mode != LBV_TRAJ_MODE_TELEOP) )
       ) {
       return -1;
    }

    if (  jjogmode &&  (ja < 0 || ja >= num_joints)) {
      fprintf(stderr,"shcom.cc: unexpected_1 %d\n",ja); return -1;
    }
    if ( !jjogmode &&  (ja < 0))                     {
      fprintf(stderr,"shcom.cc: unexpected_2 %d\n",ja); return -1;
    }

    lbv_jog_stop_msg.jjogmode = jjogmode;
    lbv_jog_stop_msg.joint_or_axis = ja;
    lbvCommandSend(lbv_jog_stop_msg);
    return 0;
}

int sendJogCont(int ja, int jjogmode, double speed)
{
    LBV_JOG_CONT lbv_jog_cont_msg;

    if (lbvStatus->task.state != LBV_TASK_STATE_ON) { return -1; }
    if (   (  (jjogmode == JOGJOINT)
            && (lbvStatus->motion.traj.mode == LBV_TRAJ_MODE_TELEOP) )
        || (   (jjogmode == JOGTELEOP )
            && (lbvStatus->motion.traj.mode != LBV_TRAJ_MODE_TELEOP) )
       ) {
       return -1;
    }

    if (  jjogmode &&  (ja < 0 || ja >= num_joints)) {
       fprintf(stderr,"shcom.cc: unexpected_3 %d\n",ja); return -1;
    }
    if ( !jjogmode &&  (ja < 0))                     {
       fprintf(stderr,"shcom.cc: unexpected_4 %d\n",ja); return -1;
    }

    lbv_jog_cont_msg.jjogmode = jjogmode;
    lbv_jog_cont_msg.joint_or_axis = ja;
    lbv_jog_cont_msg.vel = speed / 60.0;

    lbvCommandSend(lbv_jog_cont_msg);

    return 0;
}

int sendJogIncr(int ja, int jjogmode, double speed, double incr)
{
    LBV_JOG_INCR lbv_jog_incr_msg;

    if (lbvStatus->task.state != LBV_TASK_STATE_ON) { return -1; }
    if (   ( (jjogmode == JOGJOINT)
        && (  lbvStatus->motion.traj.mode == LBV_TRAJ_MODE_TELEOP) )
        || ( (jjogmode == JOGTELEOP )
        && (  lbvStatus->motion.traj.mode != LBV_TRAJ_MODE_TELEOP) )
       ) {
       return -1;
    }

    if (  jjogmode &&  (ja < 0 || ja >= num_joints)) {
        fprintf(stderr,"shcom.cc: unexpected_5 %d\n",ja); return -1;
    }
    if ( !jjogmode &&  (ja < 0))                     {
        fprintf(stderr,"shcom.cc: unexpected_6 %d\n",ja); return -1;
    }

    lbv_jog_incr_msg.jjogmode = jjogmode;
    lbv_jog_incr_msg.joint_or_axis = ja;
    lbv_jog_incr_msg.vel = speed / 60.0;
    lbv_jog_incr_msg.incr = incr;

    lbvCommandSend(lbv_jog_incr_msg);

    return 0;
}

int sendMistOn()
{
    LBV_COOLANT_MIST_ON lbv_coolant_mist_on_msg;

    lbvCommandSend(lbv_coolant_mist_on_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendMistOff()
{
    LBV_COOLANT_MIST_OFF lbv_coolant_mist_off_msg;

    lbvCommandSend(lbv_coolant_mist_off_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendFloodOn()
{
    LBV_COOLANT_FLOOD_ON lbv_coolant_flood_on_msg;

    lbvCommandSend(lbv_coolant_flood_on_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendFloodOff()
{
    LBV_COOLANT_FLOOD_OFF lbv_coolant_flood_off_msg;

    lbvCommandSend(lbv_coolant_flood_off_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendLubeOn()
{
    LBV_LUBE_ON lbv_lube_on_msg;

    lbvCommandSend(lbv_lube_on_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendLubeOff()
{
    LBV_LUBE_OFF lbv_lube_off_msg;

    lbvCommandSend(lbv_lube_off_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSpindleForward(int spindle)
{
    LBV_SPINDLE_ON lbv_spindle_on_msg;
    lbv_spindle_on_msg.spindle = spindle;
    if (lbvStatus->task.activeSettings[2] != 0) {
	lbv_spindle_on_msg.speed = fabs(lbvStatus->task.activeSettings[2]);
    } else {
	lbv_spindle_on_msg.speed = +500;
    }
    lbvCommandSend(lbv_spindle_on_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSpindleReverse(int spindle)
{
    LBV_SPINDLE_ON lbv_spindle_on_msg;
    lbv_spindle_on_msg.spindle = spindle;
    if (lbvStatus->task.activeSettings[2] != 0) {
	lbv_spindle_on_msg.speed =
	    -1 * fabs(lbvStatus->task.activeSettings[2]);
    } else {
	lbv_spindle_on_msg.speed = -500;
    }
    lbvCommandSend(lbv_spindle_on_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSpindleOff(int spindle)
{
    LBV_SPINDLE_OFF lbv_spindle_off_msg;
    lbv_spindle_off_msg.spindle = spindle;
    lbvCommandSend(lbv_spindle_off_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSpindleIncrease(int spindle)
{
    LBV_SPINDLE_INCREASE lbv_spindle_increase_msg;
    lbv_spindle_increase_msg.spindle = spindle;
    lbvCommandSend(lbv_spindle_increase_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSpindleDecrease(int spindle)
{
    LBV_SPINDLE_DECREASE lbv_spindle_decrease_msg;
    lbv_spindle_decrease_msg.spindle = spindle;
    lbvCommandSend(lbv_spindle_decrease_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSpindleConstant(int spindle)
{
    LBV_SPINDLE_CONSTANT lbv_spindle_constant_msg;
    lbv_spindle_constant_msg.spindle = spindle;
    lbvCommandSend(lbv_spindle_constant_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendBrakeEngage(int spindle)
{
    LBV_SPINDLE_BRAKE_ENGAGE lbv_spindle_brake_engage_msg;

    lbv_spindle_brake_engage_msg.spindle = spindle;
    lbvCommandSend(lbv_spindle_brake_engage_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendBrakeRelease(int spindle)
{
    LBV_SPINDLE_BRAKE_RELEASE lbv_spindle_brake_release_msg;

    lbv_spindle_brake_release_msg.spindle = spindle;
    lbvCommandSend(lbv_spindle_brake_release_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendAbort()
{
    LBV_TASK_ABORT task_abort_msg;

    lbvCommandSend(task_abort_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendHome(int joint)
{
    LBV_JOINT_HOME lbv_joint_home_msg;

    lbv_joint_home_msg.joint = joint;
    lbvCommandSend(lbv_joint_home_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendUnHome(int joint)
{
    LBV_JOINT_UNHOME lbv_joint_home_msg;

    lbv_joint_home_msg.joint = joint;
    lbvCommandSend(lbv_joint_home_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendFeedOverride(double override)
{
    LBV_TRAJ_SET_SCALE lbv_traj_set_scale_msg;

    if (override < 0.0) {
	override = 0.0;
    }

    lbv_traj_set_scale_msg.scale = override;
    lbvCommandSend(lbv_traj_set_scale_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendRapidOverride(double override)
{
    LBV_TRAJ_SET_RAPID_SCALE lbv_traj_set_scale_msg;

    if (override < 0.0) {
	override = 0.0;
    }

    if (override > 1.0) {
	override = 1.0;
    }

    lbv_traj_set_scale_msg.scale = override;
    lbvCommandSend(lbv_traj_set_scale_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}


int sendSpindleOverride(int spindle, double override)
{
    LBV_TRAJ_SET_SPINDLE_SCALE lbv_traj_set_spindle_scale_msg;

    if (override < 0.0) {
	override = 0.0;
    }

    lbv_traj_set_spindle_scale_msg.spindle = spindle;
    lbv_traj_set_spindle_scale_msg.scale = override;
    lbvCommandSend(lbv_traj_set_spindle_scale_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendTaskPlanInit()
{
    LBV_TASK_PLAN_INIT task_plan_init_msg;

    lbvCommandSend(task_plan_init_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

// saved value of last program opened
static char lastProgramFile[LINELEN] = "";

int sendProgramOpen(char *program)
{
    LBV_TASK_PLAN_OPEN lbv_task_plan_open_msg;

    // save this to run again
    strcpy(lastProgramFile, program);

    strcpy(lbv_task_plan_open_msg.file, program);
    lbvCommandSend(lbv_task_plan_open_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendProgramRun(int line)
{
    LBV_TASK_PLAN_RUN lbv_task_plan_run_msg;

    if (lbvUpdateType == LBV_UPDATE_AUTO) {
	updateStatus();
    }
    // first reopen program if it's not open
    if (0 == lbvStatus->task.file[0]) {
	// send a request to open last one
	sendProgramOpen(lastProgramFile);
    }
    // save the start line, to compare against active line later
    programStartLine = line;

    lbv_task_plan_run_msg.line = line;
    lbvCommandSend(lbv_task_plan_run_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendProgramPause()
{
    LBV_TASK_PLAN_PAUSE lbv_task_plan_pause_msg;

    lbvCommandSend(lbv_task_plan_pause_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendProgramResume()
{
    LBV_TASK_PLAN_RESUME lbv_task_plan_resume_msg;

    lbvCommandSend(lbv_task_plan_resume_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSetOptionalStop(bool state)
{
    LBV_TASK_PLAN_SET_OPTIONAL_STOP lbv_task_plan_set_optional_stop_msg;

    lbv_task_plan_set_optional_stop_msg.state = state;
    lbvCommandSend(lbv_task_plan_set_optional_stop_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}


int sendProgramStep()
{
    LBV_TASK_PLAN_STEP lbv_task_plan_step_msg;

    // clear out start line, if we had a verify before it would be -1
    programStartLine = 0;

    lbvCommandSend(lbv_task_plan_step_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendMdiCmd(const char *mdi)
{
    LBV_TASK_PLAN_EXECUTE lbv_task_plan_execute_msg;

    strcpy(lbv_task_plan_execute_msg.command, mdi);
    lbvCommandSend(lbv_task_plan_execute_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendLoadToolTable(const char *file)
{
    LBV_TOOL_LOAD_TOOL_TABLE lbv_tool_load_tool_table_msg;

    strcpy(lbv_tool_load_tool_table_msg.file, file);
    lbvCommandSend(lbv_tool_load_tool_table_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendToolSetOffset(int toolno, double zoffset, double diameter)
{
    LBV_TOOL_SET_OFFSET lbv_tool_set_offset_msg;

    lbv_tool_set_offset_msg.toolno = toolno;
    lbv_tool_set_offset_msg.offset.tran.z = zoffset;
    lbv_tool_set_offset_msg.diameter = diameter;
    lbv_tool_set_offset_msg.orientation = 0; // mill style tool table

    lbvCommandSend(lbv_tool_set_offset_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendToolSetOffset(int toolno, double zoffset, double xoffset, 
                      double diameter, double frontangle, double backangle,
                      int orientation)
{
    LBV_TOOL_SET_OFFSET lbv_tool_set_offset_msg;

    lbv_tool_set_offset_msg.toolno = toolno;
    lbv_tool_set_offset_msg.offset.tran.z = zoffset;
    lbv_tool_set_offset_msg.offset.tran.x = xoffset;
    lbv_tool_set_offset_msg.diameter = diameter;      
    lbv_tool_set_offset_msg.frontangle = frontangle;  
    lbv_tool_set_offset_msg.backangle = backangle;    
    lbv_tool_set_offset_msg.orientation = orientation;

    lbvCommandSend(lbv_tool_set_offset_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendJointSetBacklash(int joint, double backlash)
{
    LBV_JOINT_SET_BACKLASH lbv_joint_set_backlash_msg;

    lbv_joint_set_backlash_msg.joint = joint;
    lbv_joint_set_backlash_msg.backlash = backlash;
    lbvCommandSend(lbv_joint_set_backlash_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendJointEnable(int joint, int val)
{
    LBV_JOINT_ENABLE lbv_joint_enable_msg;
    LBV_JOINT_DISABLE lbv_joint_disable_msg;

    if (val) {
	lbv_joint_enable_msg.joint = joint;
	lbvCommandSend(lbv_joint_enable_msg);
    } else {
	lbv_joint_disable_msg.joint = joint;
	lbvCommandSend(lbv_joint_disable_msg);
    }
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendJointLoadComp(int joint, const char *file, int type)
{
    LBV_JOINT_LOAD_COMP lbv_joint_load_comp_msg;

    strcpy(lbv_joint_load_comp_msg.file, file);
    lbv_joint_load_comp_msg.type = type;
    lbvCommandSend(lbv_joint_load_comp_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendSetTeleopEnable(int enable)
{
    LBV_TRAJ_SET_TELEOP_ENABLE lbv_set_teleop_enable_msg;

    lbv_set_teleop_enable_msg.enable = enable;
    lbvCommandSend(lbv_set_teleop_enable_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendClearProbeTrippedFlag()
{
    LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG lbv_clear_probe_tripped_flag_msg;

    lbv_clear_probe_tripped_flag_msg.serial_number =
	lbvCommandSend(lbv_clear_probe_tripped_flag_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int sendProbe(double x, double y, double z)
{
    LBV_TRAJ_PROBE lbv_probe_msg;

    lbv_probe_msg.pos.tran.x = x;
    lbv_probe_msg.pos.tran.y = y;
    lbv_probe_msg.pos.tran.z = z;

    lbvCommandSend(lbv_probe_msg);
    if (lbvWaitType == LBV_WAIT_RECEIVED) {
	return lbvCommandWaitReceived();
    } else if (lbvWaitType == LBV_WAIT_DONE) {
	return lbvCommandWaitDone();
    }

    return 0;
}

int iniLoad(const char *filename)
{
    IniFile inifile;
    const char *inistring;
    char displayString[LINELEN] = "";
    int t;
    int i;

    // open it
    if (inifile.Open(filename) == false) {
	return -1;
    }

    if (NULL != (inistring = inifile.Find("DEBUG", "LBV"))) {
	// copy to global
	if (1 != sscanf(inistring, "%i", &lbv_debug)) {
	    lbv_debug = 0;
	}
    } else {
	// not found, use default
	lbv_debug = 0;
    }

    if (NULL != (inistring = inifile.Find("NML_FILE", "LBV"))) {
	// copy to global
	strcpy(lbv_nmlfile, inistring);
    } else {
	// not found, use default
    }

    for (t = 0; t < LBVMOT_MAX_JOINTS; t++) {
	jogPol[t] = 1;		// set to default
	sprintf(displayString, "JOINT_%d", t);
	if (NULL != (inistring =
		     inifile.Find("JOGGING_POLARITY", displayString)) &&
	    1 == sscanf(inistring, "%d", &i) && i == 0) {
	    // it read as 0, so override default
	    jogPol[t] = 0;
	}
    }

    if (NULL != (inistring = inifile.Find("LINEAR_UNITS", "DISPLAY"))) {
	if (!strcmp(inistring, "AUTO")) {
	    linearUnitConversion = LINEAR_UNITS_AUTO;
	} else if (!strcmp(inistring, "INCH")) {
	    linearUnitConversion = LINEAR_UNITS_INCH;
	} else if (!strcmp(inistring, "MM")) {
	    linearUnitConversion = LINEAR_UNITS_MM;
	} else if (!strcmp(inistring, "CM")) {
	    linearUnitConversion = LINEAR_UNITS_CM;
	}
    } else {
	// not found, leave default alone
    }

    if (NULL != (inistring = inifile.Find("ANGULAR_UNITS", "DISPLAY"))) {
	if (!strcmp(inistring, "AUTO")) {
	    angularUnitConversion = ANGULAR_UNITS_AUTO;
	} else if (!strcmp(inistring, "DEG")) {
	    angularUnitConversion = ANGULAR_UNITS_DEG;
	} else if (!strcmp(inistring, "RAD")) {
	    angularUnitConversion = ANGULAR_UNITS_RAD;
	} else if (!strcmp(inistring, "GRAD")) {
	    angularUnitConversion = ANGULAR_UNITS_GRAD;
	}
    } else {
	// not found, leave default alone
    }

    // close it
    inifile.Close();

    return 0;
}

int checkStatus ()
{
    if (lbvStatus) return 1;    
    return 0;
}



