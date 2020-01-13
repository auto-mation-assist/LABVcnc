/********************************************************************
* Description: command.c
*   lbvmotCommandhandler() takes commands passed from user space and
*   performs various functions based on the value in lbvmotCommand->command.
*   For the full list, see the LBVMOT_COMMAND enum in motion.h
*
* pc says:
*
*   Most of the configs would be better off being passed via an ioctl
*   implimentation leaving pure realtime data to be handled by
*   lbvmotCommandHandler() - This would provide a small performance
*   increase on slower systems.
*
* jmk says:
*
*   Using commands to set config parameters is "undesireable", because
*   of the large amount of code needed for each parameter.  Today you
*   need to do the following to add a single new parameter called foo:
*
*   1)  Add a member 'foo' to the config or joint structure in motion.h
*   2)  Add a command 'LBVMOT_SET_FOO" to the cmd_code_t enum in motion.h
*   3)  Add a field to the command_t struct for the value used by
*       the set command (if there isn't already one that can be used.)
*   4)  Add a case to the giant switch statement in command.c to
*       handle the 'LBVMOT_SET_FOO' command.
*   5)  Write a function lbvSetFoo() in taskintf.cc to issue the command.
*   6)  Add a prototype for lbvSetFoo() to lbv.hh
*   7)  Add code to iniaxis.cc (or one of the other inixxx.cc files) to
*       get the value from the ini file and call lbvSetFoo().  (Note
*       that each parameter has about 16 lines of code, but the code
*       is identical except for variable/parameter names.)
*   8)  Add more code to iniaxis.cc to write the new value back out
*       to the ini file.
*   After all that, you have the abililty to get a number from the
*   ini file to a structure in shared memory where the motion controller
*   can actually use it.  However, if you want to manipulate that number
*   using NML, you have to do more:
*   9)  Add a #define LBV_SET_FOO_TYPE to lbv.hh
*   10) Add a class definition for LBV_SET_FOO to lbv.hh
*   11) Add a case to a giant switch statement in lbvtaskmain.cc to
*       call lbvSetFoo() when the NML command is received.  (Actually
*       there are about 6 switch statements that need at least a
*       case label added.
*   12) Add cases to two giant switch statements in lbv.cc, associated
*       with looking up and formating the command.
*
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include <float.h>
#include "posemath.h"
#include "rtapi.h"
#include "hal.h"
#include "motion.h"
#include "motion_debug.h"
#include "motion_struct.h"
#include "mot_priv.h"
#include "rtapi_math.h"
#include "motion_types.h"
#include "homing.h"

#include "tp_debug.h"

#define ABS(x) (((x) < 0) ? -(x) : (x))

// Mark strings for translation, but defer translation to userspace
#define _(s) (s)

extern int motion_num_spindles;

static int rehomeAll;

/* loops through the active joints and checks if any are not homed */
bool checkAllHomed(void)
{
    int joint_num;
    lbvmot_joint_t *joint;

    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	joint = &joints[joint_num];
	if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	    /* if joint is not active, don't even look at its limits */
	    continue;
	}
	if (!get_homed(joint_num) ) {
	    /* if any of the joints is not homed return false */
	    return 0;
	}
    }
    /* return true if all active joints are homed*/
    return 1;
}

/* limits_ok() returns 1 if none of the hard limits are set,
   0 if any are set. Called on a linear and circular move. */
STATIC int limits_ok(void)
{
    int joint_num;
    lbvmot_joint_t *joint;

    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint data */
	joint = &joints[joint_num];
	if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	    /* if joint is not active, don't even look at its limits */
	    continue;
	}

	if (GET_JOINT_PHL_FLAG(joint) || GET_JOINT_NHL_FLAG(joint)) {
	    return 0;
	}
    }

    return 1;
}

/* check the value of the joint and velocity against current position,
   returning 1 (okay) if the request is to jog off the limit, 0 (bad)
   if the request is to jog further past a limit. */
STATIC int joint_jog_ok(int joint_num, double vel)
{
    lbvmot_joint_t *joint;
    int neg_limit_override, pos_limit_override;

    /* point to joint data */
    joint = &joints[joint_num];
    /* are any limits for this joint overridden? */
    neg_limit_override = lbvmotStatus->overrideLimitMask & ( 1 << (joint_num*2));
    pos_limit_override = lbvmotStatus->overrideLimitMask & ( 2 << (joint_num*2));
    if ( neg_limit_override && pos_limit_override ) {
	/* both limits have been overridden at the same time.  This
	   happens only when they both share an input, but means it
	   is impossible to know which direction is safe to move.  So
	   we skip the following tests... */
	return 1;
    }
    if (joint_num < 0 || joint_num >= ALL_JOINTS) {
	reportError(_("Can't jog invalid joint number %d."), joint_num);
	return 0;
    }
    if (vel > 0.0 && GET_JOINT_PHL_FLAG(joint)) {
	reportError(_("Can't jog joint %d further past max hard limit."),
	    joint_num);
	return 0;
    }
    if (vel < 0.0 && GET_JOINT_NHL_FLAG(joint)) {
	reportError(_("Can't jog joint %d further past min hard limit."),
	    joint_num);
	return 0;
    }
    refresh_jog_limits(joint,joint_num);
    if ( vel > 0.0 && (joint->pos_cmd > joint->max_jog_limit) ) {
	reportError(_("Can't jog joint %d further past max soft limit."),
	    joint_num);
	return 0;
    }
    if ( vel < 0.0 && (joint->pos_cmd < joint->min_jog_limit) ) {
	reportError(_("Can't jog joint %d further past min soft limit."),
	    joint_num);
	return 0;
    }
    /* okay to jog */
    return 1;
}

/* Jogs limits change, based on whether the machine is homed or
   or not.  If not homed, the limits are relative to the current
   position by +/- the full range of travel.  Once homed, they
   are absolute.

   homing api requires joint_num
*/
void refresh_jog_limits(lbvmot_joint_t *joint, int joint_num)
{
    double range;

    if (get_homed(joint_num) ) {
	/* if homed, set jog limits using soft limits */
	joint->max_jog_limit = joint->max_pos_limit;
	joint->min_jog_limit = joint->min_pos_limit;
    } else {
	/* not homed, set limits based on current position */
	range = joint->max_pos_limit - joint->min_pos_limit;
	joint->max_jog_limit = joint->pos_fb + range;
	joint->min_jog_limit = joint->pos_fb - range;
    }
}

static int check_axis_constraint(double target, int id, char *move_type,
                                 int axis_no, char axis_name) {
    int in_range = 1;
    double nl = axes[axis_no].min_pos_limit;
    double pl = axes[axis_no].max_pos_limit;

    double eps = 1e-308;

    if (    (fabs(target) < eps)
         && (fabs(axes[axis_no].min_pos_limit) < eps)
         && (fabs(axes[axis_no].max_pos_limit) < eps) ) { return 1;}

    if(target < nl) {
        in_range = 0;
        reportError(_("%s move on line %d would exceed %c's %s limit"),
                    move_type, id, axis_name, _("negative"));
    }

    if(target > pl) {
        in_range = 0;
        reportError(_("%s move on line %d would exceed %c's %s limit"),
                    move_type, id, axis_name, _("positive"));
    }

    return in_range;
}

/* inRange() returns non-zero if the position lies within the joint
   limits, or 0 if not.  It also reports an error for each joint limit
   violation.  It's possible to get more than one violation per move. */
STATIC int inRange(LbvPose pos, int id, char *move_type)
{
    double joint_pos[LBVMOT_MAX_JOINTS];
    int joint_num;
    lbvmot_joint_t *joint;
    int in_range = 1;

    if(check_axis_constraint(pos.tran.x, id, move_type, 0, 'X') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.tran.y, id, move_type, 1, 'Y') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.tran.z, id, move_type, 2, 'Z') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.a, id, move_type, 3, 'A') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.b, id, move_type, 4, 'B') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.c, id, move_type, 5, 'C') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.u, id, move_type, 6, 'U') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.v, id, move_type, 7, 'V') == 0) 
        in_range = 0;
    if(check_axis_constraint(pos.w, id, move_type, 8, 'W') == 0) 
        in_range = 0;

    /* Now, check that the endpoint puts the joints within their limits too */

    /* fill in all joints with 0 */
    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	joint_pos[joint_num] = 0.0;
    }

    /* now fill in with real values, for joints that are used */
    if (kinematicsInverse(&pos, joint_pos, &iflags, &fflags) < 0)
    {
	reportError(_("%s move on line %d fails kinematicsInverse"),
		    move_type, id);
	return 0;
    }

    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
	/* point to joint data */
	joint = &joints[joint_num];

	if (!GET_JOINT_ACTIVE_FLAG(joint)) {
	    /* if joint is not active, don't even look at its limits */
	    continue;
	}
	if(!isfinite(joint_pos[joint_num]))
	{
	    reportError(_("%s move on line %d gave non-finite joint location on joint %d"),
		    move_type, id, joint_num);
	    in_range = 0;
	    continue;
	}
	if (joint_pos[joint_num] > joint->max_pos_limit) {
            in_range = 0;
	    reportError(_("%s move on line %d would exceed joint %d's positive limit"),
			move_type, id, joint_num);
        }

        if (joint_pos[joint_num] < joint->min_pos_limit) {
	    in_range = 0;
	    reportError(_("%s move on line %d would exceed joint %d's negative limit"),
			move_type, id, joint_num);
	}
    }
    return in_range;
}

/* legacy note:
   clearHomes() will clear the homed flags for joints that have moved
   since homing, outside coordinated control, for machines with no
   forward kinematics. This is used in conjunction with the rehomeAll
   flag, which is set for any coordinated move that in general will
   result in all joints moving. The flag is consulted whenever a joint
   is jogged in joint mode, so that either its flag can be cleared if
   no other joints have moved, or all have to be cleared.

   NOTE: dubious usefulness (inverse-only kins etc.)
*/
void clearHomes(int joint_num)
{
    int n;
    if (lbvmotConfig->kinType == KINEMATICS_INVERSE_ONLY) {
	if (rehomeAll) {
	    for (n = 0; n < ALL_JOINTS; n++) {
                set_joint_homed(joint_num,0);
	    }
	} else {
            set_joint_homed(joint_num,0);
	}
    }
}

void lbvmotSetRotaryUnlock(int jnum, int unlock) {
    if (NULL == lbvmot_hal_data->joint[jnum].unlock) {
        reportError(
        "lbvmotSetRotaryUnlock(): No unlock pin configured for joint %d\n"
        "   Use motmod parameter: unlock_joints_mask=%X",
        jnum,1<<jnum);
        return;
    }
    *(lbvmot_hal_data->joint[jnum].unlock) = unlock;
}

int lbvmotGetRotaryIsUnlocked(int jnum) {
    static int gave_message = 0;
    if (NULL == lbvmot_hal_data->joint[jnum].unlock) {
        if (!gave_message) {
            reportError(
            "lbvmotGetRotaryUnlocked(): No unlock pin configured for joint %d\n"
            "   Use motmod parameter: unlock_joints_mask=%X'",
            jnum,1<<jnum);
        }
        gave_message = 1;
        return 0;
    }
    return *(lbvmot_hal_data->joint[jnum].is_unlocked);
}

/*! \function lbvmotDioWrite()

  sets or clears a HAL DIO pin, 
  pins get exported at runtime
  
  index is valid from 0 to lbvmotConfig->num_dio <= LBVMOT_MAX_DIO, defined in lbvmotcfg.h
  
*/
void lbvmotDioWrite(int index, char value)
{
    if ((index >= lbvmotConfig->numDIO) || (index < 0)) {
	rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: index out of range, %d not in [0..%d] (increase num_dio/LBVMOT_MAX_DIO=%d)\n", index, lbvmotConfig->numDIO, LBVMOT_MAX_DIO);
    } else {
	if (value != 0) {
	    *(lbvmot_hal_data->synch_do[index])=1;
	} else {
	    *(lbvmot_hal_data->synch_do[index])=0;
	}
    }
}

/*! \function lbvmotAioWrite()

  sets or clears a HAL AIO pin, 
  pins get exported at runtime
  
  index is valid from 0 to lbvmotConfig->num_aio <= LBVMOT_MAX_AIO, defined in lbvmotcfg.h
  
*/
void lbvmotAioWrite(int index, double value)
{
    if ((index >= lbvmotConfig->numAIO) || (index < 0)) {
	rtapi_print_msg(RTAPI_MSG_ERR, "ERROR: index out of range, %d not in [0..%d] (increase num_aio/LBVMOT_MAX_AIO=%d)\n", index, lbvmotConfig->numAIO, LBVMOT_MAX_AIO);
    } else {
        *(lbvmot_hal_data->analog_output[index]) = value;
    }
}

STATIC int is_feed_type(int motion_type)
{
    switch(motion_type) {
    case LBV_MOTION_TYPE_ARC:
    case LBV_MOTION_TYPE_FEED:
    case LBV_MOTION_TYPE_PROBING:
        return 1;
    default:
        rtapi_print_msg(RTAPI_MSG_ERR, "Internal error: unhandled motion type %d\n", motion_type);
    case LBV_MOTION_TYPE_TOOLCHANGE:
    case LBV_MOTION_TYPE_TRAVERSE:
    case LBV_MOTION_TYPE_INDEXROTARY:
        return 0;
    }
}

/*
  lbvmotCommandHandler() is called each main cycle to read the
  shared memory buffer
  */
void lbvmotCommandHandler(void *arg, long period)
{
    int joint_num, axis_num, spindle_num;
    int n;
    lbvmot_joint_t *joint;
    lbvmot_axis_t *axis;
    double tmp1;
    lbvmot_comp_entry_t *comp_entry;
    char issue_atspeed = 0;
    int abort = 0;
    char* emsg = "";

    /* check for split read */
    if (lbvmotCommand->head != lbvmotCommand->tail) {
	lbvmotDebug->split++;
	return;			/* not really an error */
    }
    if (lbvmotCommand->commandNum != lbvmotStatus->commandNumEcho) {
	/* increment head count-- we'll be modifying lbvmotStatus */
	lbvmotStatus->head++;
	lbvmotDebug->head++;

	/* got a new command-- echo command and number... */
	lbvmotStatus->commandEcho = lbvmotCommand->command;
	lbvmotStatus->commandNumEcho = lbvmotCommand->commandNum;

	/* clear status value by default */
	lbvmotStatus->commandStatus = LBVMOT_COMMAND_OK;
	
	/* ...and process command */

        joint = 0;
        axis  = 0;
        joint_num = lbvmotCommand->joint;
        axis_num  = lbvmotCommand->axis;

//-----------------------------------------------------------------------------
// joints_axes test for unexpected conditions
// example: non-cooperating guis
// example: attempt to jog locking indexer axis letter
        if (   lbvmotCommand->command == LBVMOT_JOG_CONT
            || lbvmotCommand->command == LBVMOT_JOG_INCR
            || lbvmotCommand->command == LBVMOT_JOG_ABS
           ) {
           if (GET_MOTION_TELEOP_FLAG() && axis_num < 0) {
               emsg = "command.com teleop: unexpected negative axis_num";
               if (joint_num >= 0) {
                   emsg = "Mode is TELEOP, cannot jog joint";
               }
               abort = 1;
           }
           if (!GET_MOTION_TELEOP_FLAG() && joint_num < 0) {
               emsg = "command.com !teleop: unexpected negative joint_num";
               if (axis_num >= 0) {
                   emsg = "Mode is NOT TELEOP, cannot jog axis coordinate";
               }
               abort = 1;
           }
           if (   !GET_MOTION_TELEOP_FLAG()
               && (joint_num >= ALL_JOINTS || joint_num <  0)
              ) {
               rtapi_print_msg(RTAPI_MSG_ERR,
                    "Joint jog requested for undefined joint number=%d (min=0,max=%d)",
                    joint_num,ALL_JOINTS-1);
               return;
           }
           if (GET_MOTION_TELEOP_FLAG()) {
                axis = &axes[axis_num];
                if ( (axis_num >= 0) && (axis->locking_joint >= 0) ) {
                    rtapi_print_msg(RTAPI_MSG_ERR,
                    "Cannot jog a locking indexer AXIS_%c,joint_num=%d\n",
                    "XYZABCUVW"[axis_num],axis->locking_joint);
                    return;
                }
           }
        }
        if (abort) {
          switch (lbvmotCommand->command) {
          case LBVMOT_JOG_CONT:
               rtapi_print_msg(RTAPI_MSG_ERR,"JOG_CONT %s\n",emsg);
               break;
          case LBVMOT_JOG_INCR:
               rtapi_print_msg(RTAPI_MSG_ERR,"JOG_INCR %s\n",emsg);
               break;
          case LBVMOT_JOG_ABS:
               rtapi_print_msg(RTAPI_MSG_ERR,"JOG_ABS %s\n",emsg);
               break;
          default: break;
          }
          return;
        }

        if (joint_num >= 0 && joint_num < ALL_JOINTS) {
            joint = &joints[joint_num];
            if (   (   lbvmotCommand->command == LBVMOT_JOG_CONT
                    || lbvmotCommand->command == LBVMOT_JOG_INCR
                    || lbvmotCommand->command == LBVMOT_JOG_ABS
                   )
                && !(GET_MOTION_TELEOP_FLAG())
                && get_home_is_synchronized(joint_num)
                && !get_homing_is_active()
               ) {
                  if (lbvmotConfig->kinType == KINEMATICS_IDENTITY) {
                      rtapi_print_msg(RTAPI_MSG_ERR,
                      "Homing is REQUIRED to jog requested coordinate\n"
                      "because joint (%d) home_sequence is synchronized (%d)\n"
                      ,joint_num,get_home_sequence(joint_num));
                  } else {
                      rtapi_print_msg(RTAPI_MSG_ERR,
                      "Cannot jog joint %d because home_sequence is synchronized (%d)\n"
                      ,joint_num,get_home_sequence(joint_num));
                  }
                  return;
            }
        }
        if (axis_num >= 0 && axis_num < LBVMOT_MAX_AXIS) {
            axis = &axes[axis_num];
        }
	switch (lbvmotCommand->command) {
	case LBVMOT_ABORT:
	    /* abort motion */
	    /* can happen at any time */
	    /* this command attempts to stop all machine motion. it looks at
	       the current mode and acts accordingly, if in teleop mode, it
	       sets the desired velocities to zero, if in coordinated mode,
	       it calls the traj planner abort function (don't know what that
	       does yet), and if in free mode, it disables the free mode traj
	       planners which stops joint motion */
	    rtapi_print_msg(RTAPI_MSG_DBG, "ABORT");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    /* check for coord or free space motion active */
	    if (GET_MOTION_TELEOP_FLAG()) {
		for (axis_num = 0; axis_num < LBVMOT_MAX_AXIS; axis_num++) {
		    /* point to joint struct */
		    axis = &axes[axis_num];
		    /* tell teleop planner to stop */
		    axis->teleop_tp.enable = 0;
                }
	    } else if (GET_MOTION_COORD_FLAG()) {
		tpAbort(&lbvmotDebug->coord_tp);
	    } else {
		for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
		    /* point to joint struct */
		    joint = &joints[joint_num];
		    /* tell joint planner to stop */
		    joint->free_tp.enable = 0;
		    /* stop homing if in progress */
		    if ( ! get_home_is_idle(joint_num)) {
			set_home_abort(joint_num);
		    }
		}
	    }
            SET_MOTION_ERROR_FLAG(0);
	    /* clear joint errors (regardless of mode) */
	    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
		/* point to joint struct */
		joint = &joints[joint_num];
		/* update status flags */
		SET_JOINT_ERROR_FLAG(joint, 0);
		SET_JOINT_FAULT_FLAG(joint, 0);
	    }
	    lbvmotStatus->paused = 0;
	    break;

	case LBVMOT_JOINT_ABORT:
	    /* abort one joint */
	    /* can happen at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_ABORT");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (GET_MOTION_TELEOP_FLAG()) {
		/* tell teleop planner to stop */
		if (axis != 0) axis->teleop_tp.enable = 0;
		/* do nothing in teleop mode */
	    } else if (GET_MOTION_COORD_FLAG()) {
		/* do nothing in coord mode */
	    } else {
		/* tell joint planner to stop */
		if (joint != 0) joint->free_tp.enable = 0;
		/* validate joint */
		if (joint == 0) { break; }
		/* stop homing if in progress */
		if ( !get_home_is_idle(joint_num) ) {
                    set_home_abort(joint_num);
		}
		/* update status flags */
		SET_JOINT_ERROR_FLAG(joint, 0);
	    }
	    break;

	case LBVMOT_FREE:
            for (axis_num = 0; axis_num < LBVMOT_MAX_AXIS; axis_num++) {
              axis = &axes[axis_num];
              if (axis != 0) { axis->teleop_tp.enable = 0; }
            }
	    /* change the mode to free mode motion (joint mode) */
	    /* can be done at any time */
	    /* this code doesn't actually make the transition, it merely
	       requests the transition by clearing a couple of flags */
	    /* reset the lbvmotDebug->coordinating flag to defer transition
	       to controller cycle */
	    rtapi_print_msg(RTAPI_MSG_DBG, "FREE");
	    lbvmotDebug->coordinating = 0;
	    lbvmotDebug->teleoperating = 0;
	    break;

	case LBVMOT_COORD:
	    /* change the mode to coordinated axis motion */
	    /* can be done at any time */
	    /* this code doesn't actually make the transition, it merely
	       tests a condition and then sets a flag requesting the
	       transition */
	    /* set the lbvmotDebug->coordinating flag to defer transition to
	       controller cycle */

	    rtapi_print_msg(RTAPI_MSG_DBG, "COORD");
	    lbvmotDebug->coordinating = 1;
	    lbvmotDebug->teleoperating = 0;
	    if (lbvmotConfig->kinType != KINEMATICS_IDENTITY) {
		if (!checkAllHomed()) {
		    reportError
			(_("all joints must be homed before going into coordinated mode"));
		    lbvmotDebug->coordinating = 0;
		    break;
		}
	    }
	    break;

	case LBVMOT_TELEOP:
	    rtapi_print_msg(RTAPI_MSG_DBG, "TELEOP");
            switch_to_teleop_mode();
	    break;

	case LBVMOT_SET_NUM_JOINTS:
	    /* set the global NUM_JOINTS, which must be between 1 and
	       LBVMOT_MAX_JOINTS, inclusive.
	       Called  by task using [KINS]JOINTS= which is typically
	       the same value as the motmod num_joints= parameter
	    */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_NUM_JOINTS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", lbvmotCommand->joint);
	    if (( lbvmotCommand->joint <= 0 ) ||
		( lbvmotCommand->joint > LBVMOT_MAX_JOINTS )) {
		break;
	    }
	    ALL_JOINTS = lbvmotCommand->joint;
	    break;

	case LBVMOT_SET_NUM_SPINDLES:
	    /* set the global NUM_SPINDLES, which must be between 1 and
	       LBVMOT_MAX_SPINDLES, inclusive and less than or equal to
	       the number of spindles configured for the motion module
	       (motion_num_spindles)
	    */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_NUM_SPINDLES");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", lbvmotCommand->spindle);
	    if (   lbvmotCommand->spindle > motion_num_spindles
	        || lbvmotCommand->spindle <= 0
	        || lbvmotCommand->spindle > LBVMOT_MAX_SPINDLES
	       ) {
	        reportError("Problem:\n"
	                    "  motmod configured for %d spindles\n"
	                    "  but command requests %d spindles\n"
	                    "  Using: %d spindles",
	                    motion_num_spindles,
	                    lbvmotCommand->spindle,
	                    motion_num_spindles
	                   );
	        lbvmotConfig->numSpindles = motion_num_spindles;
	    } else {
	        lbvmotConfig->numSpindles = lbvmotCommand->spindle;
	    }
	    break;

	case LBVMOT_SET_WORLD_HOME:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_WORLD_HOME");
	    lbvmotStatus->world_home = lbvmotCommand->pos;
	    break;

	case LBVMOT_SET_JOINT_HOMING_PARAMS:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_HOMING_PARAMS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    if (joint == 0) {
		break;
	    }
	    set_joint_homing_params(joint_num,
	                            lbvmotCommand->offset,
	                            lbvmotCommand->home,
	                            lbvmotCommand->home_final_vel,
	                            lbvmotCommand->search_vel,
	                            lbvmotCommand->latch_vel,
	                            lbvmotCommand->flags,
	                            lbvmotCommand->home_sequence,
	                            lbvmotCommand->volatile_home
	                           );
	    break;

	case LBVMOT_UPDATE_JOINT_HOMING_PARAMS:
	    rtapi_print_msg(RTAPI_MSG_DBG, "UPDATE_JOINT_HOMING_PARAMS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    if (joint == 0) {
		break;
	    }
	    update_joint_homing_params(joint_num,
	                               lbvmotCommand->offset,
	                               lbvmotCommand->home,
	                               lbvmotCommand->home_sequence
	                               );
	    break;

	case LBVMOT_OVERRIDE_LIMITS:
	    /* this command can be issued with joint < 0 to re-enable
	       limits, but they are automatically re-enabled at the
	       end of the next jog */
	    rtapi_print_msg(RTAPI_MSG_DBG, "OVERRIDE_LIMITS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (joint_num < 0) {
		/* don't override limits */
		rtapi_print_msg(RTAPI_MSG_DBG, "override off");
		lbvmotStatus->overrideLimitMask = 0;
	    } else {
		rtapi_print_msg(RTAPI_MSG_DBG, "override on");
		lbvmotStatus->overrideLimitMask = 0;
		for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
		    /* point at joint data */
		    joint = &joints[joint_num];
		    /* only override limits that are currently tripped */
		    if ( GET_JOINT_NHL_FLAG(joint) ) {
			lbvmotStatus->overrideLimitMask |= (1 << (joint_num*2));
		    }
		    if ( GET_JOINT_PHL_FLAG(joint) ) {
			lbvmotStatus->overrideLimitMask |= (2 << (joint_num*2));
		    }
		}
	    }
	    lbvmotDebug->overriding = 0;
	    for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
		/* point at joint data */
		joint = &joints[joint_num];
		/* clear joint errors */
		SET_JOINT_ERROR_FLAG(joint, 0);
	    }
	    break;

	case LBVMOT_SET_JOINT_MOTOR_OFFSET:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_MOTOR_OFFSET");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if(joint == 0) {
		break;
	    }
	    joint->motor_offset = lbvmotCommand->motor_offset;
	    break;

	case LBVMOT_SET_JOINT_POSITION_LIMITS:
	    /* set the position limits for the joint */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_POSITION_LIMITS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    if (joint == 0) {
		break;
	    }
	    joint->min_pos_limit = lbvmotCommand->minLimit;
	    joint->max_pos_limit = lbvmotCommand->maxLimit;
	    break;

	case LBVMOT_SET_JOINT_BACKLASH:
	    /* set the backlash for the joint */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_BACKLASH");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    if (joint == 0) {
		break;
	    }
	    joint->backlash = lbvmotCommand->backlash;
	    break;

	    /*
	       Max and min ferror work like this: limiting ferror is
	       determined by slope of ferror line, = maxFerror/limitVel ->
	       limiting ferror = maxFerror/limitVel * vel. If ferror <
	       minFerror then OK else if ferror < limiting ferror then OK
	       else ERROR */
	case LBVMOT_SET_JOINT_MAX_FERROR:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_MAX_FERROR");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    if (joint == 0 || lbvmotCommand->maxFerror < 0.0) {
		break;
	    }
	    joint->max_ferror = lbvmotCommand->maxFerror;
	    break;

	case LBVMOT_SET_JOINT_MIN_FERROR:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_MIN_FERROR");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    if (joint == 0 || lbvmotCommand->minFerror < 0.0) {
		break;
	    }
	    joint->min_ferror = lbvmotCommand->minFerror;
	    break;

	case LBVMOT_JOG_CONT:
	    /* do a continuous jog, implemented as an incremental jog to the
	       limit.  When the user lets go of the button an abort will
	       stop the jog. */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_CONT");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (!GET_MOTION_ENABLE_FLAG()) {
		reportError(_("Can't jog joint when not enabled."));
		SET_JOINT_ERROR_FLAG(joint, 1);
		break;
	    }
	    if ( get_homing_is_active() ) {
		reportError(_("Can't jog any joints while homing."));
		SET_JOINT_ERROR_FLAG(joint, 1);
		break;
	    }
	    if (lbvmotStatus->net_feed_scale < 0.0001) {
		/* don't jog if feedhold is on or if feed override is zero */
		break;
	    }
            if (!GET_MOTION_TELEOP_FLAG()) {
	        if (joint->wheel_jjog_active) {
		    /* can't do two kinds of jog at once */
		    break;
	        }
                if (get_home_needs_unlock_first(joint_num) ) {
                    reportError("Can't jog locking joint_num=%d",joint_num);
                    SET_JOINT_ERROR_FLAG(joint, 1);
                    break;
                }
	        /* don't jog further onto limits */
	        if (!joint_jog_ok(joint_num, lbvmotCommand->vel)) {
		    SET_JOINT_ERROR_FLAG(joint, 1);
		    break;
	        }
	        /* set destination of jog */
	        refresh_jog_limits(joint,joint_num);
	        if (lbvmotCommand->vel > 0.0) {
		    joint->free_tp.pos_cmd = joint->max_jog_limit;
	        } else {
		    joint->free_tp.pos_cmd = joint->min_jog_limit;
	        }
	        /* set velocity of jog */
	        joint->free_tp.max_vel = fabs(lbvmotCommand->vel);
	        /* use max joint accel */
	        joint->free_tp.max_acc = joint->acc_limit;
	        /* lock out other jog sources */
	        joint->kb_jjog_active = 1;
	        /* and let it go */
	        joint->free_tp.enable = 1;
                for (axis_num = 0; axis_num < LBVMOT_MAX_AXIS; axis_num++) {
                    axis = &axes[axis_num];
                    if (axis != 0) { axis->teleop_tp.enable = 0; }
                }
	        /*! \todo FIXME - should we really be clearing errors here? */
	        SET_JOINT_ERROR_FLAG(joint, 0);
	        /* clear joints homed flag(s) if we don't have forward kins.
	           Otherwise, a transition into coordinated mode will incorrectly
	           assume the homed position. Do all if they've all been moved
	           since homing, otherwise just do this one */
	        clearHomes(joint_num);
            } else {
                // TELEOP  JOG_CONT
                if (GET_MOTION_ERROR_FLAG()) { break; }
                axis_hal_t *axis_data = &(lbvmot_hal_data->axis[axis_num]);
                if (   axis->ext_offset_tp.enable
                    && (fabs(*(axis_data->external_offset)) > EOFFSET_EPSILON)) {
                    /* here: set pos_cmd to a big number so that with combined
                    *        teleop jog plus external offsets the soft limits
                    *        can always be reached
                    *  a fixed epsilon is used here for convenience
                    *  it is not the same as the epsilon used as a stopping 
                    *  criterion in control.c
                    */
                    if (lbvmotCommand->vel > 0.0) {
                        axis->teleop_tp.pos_cmd =  1e12; // 1T halscope limit
                    } else {
                        axis->teleop_tp.pos_cmd = -1e12; // 1T halscope limit
                    }
                } else {
                    if (lbvmotCommand->vel > 0.0) {
                        axis->teleop_tp.pos_cmd = axis->max_pos_limit;
                    } else {
                        axis->teleop_tp.pos_cmd = axis->min_pos_limit;
                    }
                }

	        axis->teleop_tp.max_vel = fabs(lbvmotCommand->vel);
	        axis->teleop_tp.max_acc = axis->acc_limit;
	        axis->kb_ajog_active = 1;
                for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
                    joint = &joints[joint_num];
                    if (joint != 0) { joint->free_tp.enable = 0; }
                }
	        axis->teleop_tp.enable = 1;
            }
	    break;

	case LBVMOT_JOG_INCR:
	    /* do an incremental jog */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_INCR");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (!GET_MOTION_ENABLE_FLAG()) {
		reportError(_("Can't jog joint when not enabled."));
		SET_JOINT_ERROR_FLAG(joint, 1);
		break;
	    }
	    if ( get_homing_is_active() ) {
		reportError(_("Can't jog any joint while homing."));
		SET_JOINT_ERROR_FLAG(joint, 1);
		break;
	    }
	    if (lbvmotStatus->net_feed_scale < 0.0001 ) {
		/* don't jog if feedhold is on or if feed override is zero */
		break;
	    }
            if (!GET_MOTION_TELEOP_FLAG()) {
	        if (joint->wheel_jjog_active) {
		    /* can't do two kinds of jog at once */
		    break;
	        }
                if (get_home_needs_unlock_first(joint_num) ) {
                    reportError("Can't jog locking joint_num=%d",joint_num);
                    SET_JOINT_ERROR_FLAG(joint, 1);
                    break;
                }
	        /* don't jog further onto limits */
	        if (!joint_jog_ok(joint_num, lbvmotCommand->vel)) {
		    SET_JOINT_ERROR_FLAG(joint, 1);
		    break;
	        }
	        /* set target position for jog */
	        if (lbvmotCommand->vel > 0.0) {
		    tmp1 = joint->free_tp.pos_cmd + lbvmotCommand->offset;
	        } else {
		    tmp1 = joint->free_tp.pos_cmd - lbvmotCommand->offset;
	        }
	        /* don't jog past limits */
	        refresh_jog_limits(joint,joint_num);
	        if (tmp1 > joint->max_jog_limit) {
		    break;
	        }
	        if (tmp1 < joint->min_jog_limit) {
		    break;
	        }
	        /* set target position */
	        joint->free_tp.pos_cmd = tmp1;
	        /* set velocity of jog */
	        joint->free_tp.max_vel = fabs(lbvmotCommand->vel);
	        /* use max joint accel */
	        joint->free_tp.max_acc = joint->acc_limit;
	        /* lock out other jog sources */
	        joint->kb_jjog_active = 1;
	        /* and let it go */
	        joint->free_tp.enable = 1;
                for (axis_num = 0; axis_num < LBVMOT_MAX_AXIS; axis_num++) {
                    axis = &axes[axis_num];
                    if (axis != 0) { axis->teleop_tp.enable = 0; }
                }
	        SET_JOINT_ERROR_FLAG(joint, 0);
	        /* clear joint homed flag(s) if we don't have forward kins.
	           Otherwise, a transition into coordinated mode will incorrectly
	           assume the homed position. Do all if they've all been moved
	           since homing, otherwise just do this one */
	        clearHomes(joint_num);
            } else {
                // TELEOP JOG_INCR
                if (GET_MOTION_ERROR_FLAG()) { break; }
	        if (lbvmotCommand->vel > 0.0) {
		    tmp1 = axis->teleop_tp.pos_cmd + lbvmotCommand->offset;
	        } else {
		    tmp1 = axis->teleop_tp.pos_cmd - lbvmotCommand->offset;
	        }
                axis_hal_t *axis_data = &(lbvmot_hal_data->axis[axis_num]);
                // a fixed epsilon is used here for convenience
                // it is not the same as the epsilon used as a stopping 
                // criterion in control.c
                if (   axis->ext_offset_tp.enable
                    && (fabs(*(axis_data->external_offset)) > EOFFSET_EPSILON)) {
                    // external_offsets: soft limit enforcement is in control.c
                } else {
                    if (tmp1 > axis->max_pos_limit) { break; }
                    if (tmp1 < axis->min_pos_limit) { break; }
                }

	        axis->teleop_tp.pos_cmd = tmp1;
	        axis->teleop_tp.max_vel = fabs(lbvmotCommand->vel);
	        axis->teleop_tp.max_acc = axis->acc_limit;
	        axis->kb_ajog_active = 1;
	        axis->teleop_tp.enable = 1;
                for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
                    joint = &joints[joint_num];
                    if (joint != 0) { joint->free_tp.enable = 0; }
                }
            }
	    break;

	case LBVMOT_JOG_ABS:
	    /* do an absolute jog */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOG_ABS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (joint == 0) {
		break;
	    }
	    if (!GET_MOTION_ENABLE_FLAG()) {
		reportError(_("Can't jog joint when not enabled."));
		SET_JOINT_ERROR_FLAG(joint, 1);
		break;
	    }
	    if ( get_homing_is_active() ) {
		reportError(_("Can't jog any joints while homing."));
		SET_JOINT_ERROR_FLAG(joint, 1);
		break;
	    }
            if (!GET_MOTION_TELEOP_FLAG()) {
                // FREE JOG_ABS
                if (joint->wheel_jjog_active) {
                    /* can't do two kinds of jog at once */
                    break;
                }
                if (lbvmotStatus->net_feed_scale < 0.0001 ) {
                    /* don't jog if feedhold is on or if feed override is zero */
                    break;
                }
                /* don't jog further onto limits */
                if (!joint_jog_ok(joint_num, lbvmotCommand->vel)) {
                    SET_JOINT_ERROR_FLAG(joint, 1);
                    break;
                }
                /*! \todo FIXME-- use 'goal' instead */
                joint->free_tp.pos_cmd = lbvmotCommand->offset;
                /* don't jog past limits */
                refresh_jog_limits(joint,joint_num);
                if (joint->free_tp.pos_cmd > joint->max_jog_limit) {
                    joint->free_tp.pos_cmd = joint->max_jog_limit;
                }
                if (joint->free_tp.pos_cmd < joint->min_jog_limit) {
                    joint->free_tp.pos_cmd = joint->min_jog_limit;
                }
                /* set velocity of jog */
                joint->free_tp.max_vel = fabs(lbvmotCommand->vel);
                /* use max joint accel */
                joint->free_tp.max_acc = joint->acc_limit;
                /* lock out other jog sources */
                joint->kb_jjog_active = 1;
                /* and let it go */
                joint->free_tp.enable = 1;
                SET_JOINT_ERROR_FLAG(joint, 0);
                /* clear joint homed flag(s) if we don't have forward kins.
                   Otherwise, a transition into coordinated mode will incorrectly
                   assume the homed position. Do all if they've all been moved
                   since homing, otherwise just do this one */
                clearHomes(joint_num);
            } else {
                axis->kb_ajog_active = 1;
                // TELEOP JOG_ABS
                if (axis->wheel_ajog_active) { break; }
	        if (lbvmotCommand->vel > 0.0) {
		    tmp1 = axis->teleop_tp.pos_cmd + lbvmotCommand->offset;
	        } else {
		    tmp1 = axis->teleop_tp.pos_cmd - lbvmotCommand->offset;
	        }
	        if (tmp1 > axis->max_pos_limit) { break; }
	        if (tmp1 < axis->min_pos_limit) { break; }
                axis->teleop_tp.pos_cmd = tmp1;
                axis->teleop_tp.max_vel = fabs(lbvmotCommand->vel);
                axis->teleop_tp.max_acc = axis->acc_limit;
                axis->kb_ajog_active = 1;
                axis->teleop_tp.enable = 1;
                for (joint_num = 0; joint_num < ALL_JOINTS; joint_num++) {
                   joint = &joints[joint_num];
                   if (joint != 0) { joint->free_tp.enable = 0; }
                }
                return;
            }
            break;

	case LBVMOT_SET_TERM_COND:
	    /* sets termination condition for motion lbvmotDebug->coord_tp */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_TERM_COND");
	    tpSetTermCond(&lbvmotDebug->coord_tp, lbvmotCommand->termCond, lbvmotCommand->tolerance);
	    break;

	case LBVMOT_SET_SPINDLESYNC:
		tpSetSpindleSync(&lbvmotDebug->coord_tp, lbvmotCommand->spindle, lbvmotCommand->spindlesync, lbvmotCommand->flags);
		break;

	case LBVMOT_SET_LINE:
	    /* lbvmotDebug->coord_tp up a linear move */
	    /* requires motion enabled, coordinated mode, not on limits */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_LINE");
	    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
		reportError(_("need to be enabled, in coord mode for linear move"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!inRange(lbvmotCommand->pos, lbvmotCommand->id, "Linear")) {
		reportError(_("invalid params in linear command"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!limits_ok()) {
		reportError(_("can't do linear move with limits exceeded"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    }

		if(lbvmotStatus->atspeed_next_feed && is_feed_type(lbvmotCommand->motion_type) ) {
			issue_atspeed = 1;
			lbvmotStatus->atspeed_next_feed = 0;
		}
		if(!is_feed_type(lbvmotCommand->motion_type) &&
				lbvmotStatus->spindle_status[lbvmotCommand->spindle].css_factor) {
			lbvmotStatus->atspeed_next_feed = 1;
		}

	    /* append it to the lbvmotDebug->coord_tp */
	    tpSetId(&lbvmotDebug->coord_tp, lbvmotCommand->id);
        int res_addline = tpAddLine(&lbvmotDebug->coord_tp, lbvmotCommand->pos, lbvmotCommand->motion_type, 
                                lbvmotCommand->vel, lbvmotCommand->ini_maxvel, 
                                lbvmotCommand->acc, lbvmotStatus->enables_new, issue_atspeed,
                                lbvmotCommand->turn);
        //KLUDGE ignore zero length line
        if (res_addline < 0) {
            reportError(_("can't add linear move at line %d, error code %d"),
                    lbvmotCommand->id, res_addline);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_BAD_EXEC;
            tpAbort(&lbvmotDebug->coord_tp);
            SET_MOTION_ERROR_FLAG(1);
            break;
        } else if (res_addline != 0) {
            //TODO make this hand-shake more explicit
            //KLUDGE Non fatal error, need to restore state so that the next
            //line properly handles at_speed
            if (issue_atspeed) {
                lbvmotStatus->atspeed_next_feed = 1;
            }
        } else {
		SET_MOTION_ERROR_FLAG(0);
		/* set flag that indicates all joints need rehoming, if any
		   joint is moved in joint mode, for machines with no forward
		   kins */
		rehomeAll = 1;
	    }
	    break;

	case LBVMOT_SET_CIRCLE:
	    /* lbvmotDebug->coord_tp up a circular move */
	    /* requires coordinated mode, enable on, not on limits */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_CIRCLE");
	    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
		reportError(_("need to be enabled, in coord mode for circular move"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!inRange(lbvmotCommand->pos, lbvmotCommand->id, "Circular")) {
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!limits_ok()) {
		reportError(_("can't do circular move with limits exceeded"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    }
            if(lbvmotStatus->atspeed_next_feed) {
                issue_atspeed = 1;
                lbvmotStatus->atspeed_next_feed = 0;
            }
	    /* append it to the lbvmotDebug->coord_tp */
	    tpSetId(&lbvmotDebug->coord_tp, lbvmotCommand->id);
	    int res_addcircle = tpAddCircle(&lbvmotDebug->coord_tp, lbvmotCommand->pos,
                            lbvmotCommand->center, lbvmotCommand->normal,
                            lbvmotCommand->turn, lbvmotCommand->motion_type,
                            lbvmotCommand->vel, lbvmotCommand->ini_maxvel,
                            lbvmotCommand->acc, lbvmotStatus->enables_new, issue_atspeed);
        if (res_addcircle < 0) {
            reportError(_("can't add circular move at line %d, error code %d"),
                    lbvmotCommand->id, res_addcircle);
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_BAD_EXEC;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
        } else if (res_addcircle != 0) {
            //FIXME! This is a band-aid for a single issue, but there may be
            //other consequences of non-fatal errors from AddXXX functions. We
            //either need to fix the root cause (subtle position error after
            //homing), or have a full restore here.
            if (issue_atspeed) {
                lbvmotStatus->atspeed_next_feed = 1;
            }
        } else {
		SET_MOTION_ERROR_FLAG(0);
		/* set flag that indicates all joints need rehoming, if any
		   joint is moved in joint mode, for machines with no forward
		   kins */
		rehomeAll = 1;
	    }
	    break;

	case LBVMOT_SET_VEL:
	    /* set the velocity for subsequent moves */
	    /* can do it at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_VEL");
	    lbvmotStatus->vel = lbvmotCommand->vel;
	    tpSetVmax(&lbvmotDebug->coord_tp, lbvmotStatus->vel, lbvmotCommand->ini_maxvel);
	    break;

	case LBVMOT_SET_VEL_LIMIT:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_VEL_LIMIT");
	    lbvmot_config_change();
	    /* set the absolute max velocity for all subsequent moves */
	    /* can do it at any time */
	    lbvmotConfig->limitVel = lbvmotCommand->vel;
	    tpSetVlimit(&lbvmotDebug->coord_tp, lbvmotConfig->limitVel);
	    break;

	case LBVMOT_SET_JOINT_VEL_LIMIT:
	    /* set joint max velocity */
	    /* can do it at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_VEL_LIMIT");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    /* check joint range */
	    if (joint == 0) {
		break;
	    }
	    joint->vel_limit = lbvmotCommand->vel;
	    break;

	case LBVMOT_SET_JOINT_ACC_LIMIT:
	    /* set joint max acceleration */
	    /* can do it at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_ACC_LIMIT");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    lbvmot_config_change();
	    /* check joint range */
	    if (joint == 0) {
		break;
	    }
	    joint->acc_limit = lbvmotCommand->acc;
	    break;

	case LBVMOT_SET_ACC:
	    /* set the max acceleration */
	    /* can do it at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_ACCEL");
	    lbvmotStatus->acc = lbvmotCommand->acc;
	    tpSetAmax(&lbvmotDebug->coord_tp, lbvmotStatus->acc);
	    break;

	case LBVMOT_PAUSE:
	    /* pause the motion */
	    /* can happen at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "PAUSE");
	    tpPause(&lbvmotDebug->coord_tp);
	    lbvmotStatus->paused = 1;
	    break;

	case LBVMOT_REVERSE:
	    /* run motion in reverse*/
	    /* only allowed during a pause */
	    rtapi_print_msg(RTAPI_MSG_DBG, "REVERSE");
	    tpSetRunDir(&lbvmotDebug->coord_tp, TC_DIR_REVERSE);
	    break;

	case LBVMOT_FORWARD:
	    /* run motion in reverse*/
	    /* only allowed during a pause */
	    rtapi_print_msg(RTAPI_MSG_DBG, "FORWARD");
	    tpSetRunDir(&lbvmotDebug->coord_tp, TC_DIR_FORWARD);
	    break;

	case LBVMOT_RESUME:
	    /* resume paused motion */
	    /* can happen at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "RESUME");
	    lbvmotDebug->stepping = 0;
	    tpResume(&lbvmotDebug->coord_tp);
	    lbvmotStatus->paused = 0;
	    break;

	case LBVMOT_STEP:
	    /* resume paused motion until id changes */
	    /* can happen at any time */
            rtapi_print_msg(RTAPI_MSG_DBG, "STEP");
            if(lbvmotStatus->paused) {
                lbvmotDebug->idForStep = lbvmotStatus->id;
                lbvmotDebug->stepping = 1;
                tpResume(&lbvmotDebug->coord_tp);
                lbvmotStatus->paused = 1;
            } else {
		reportError(_("MOTION: can't STEP while already executing"));
	    }
	    break;

	case LBVMOT_FEED_SCALE:
	    /* override speed */
	    /* can happen at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "FEED SCALE");
	    if (lbvmotCommand->scale < 0.0) {
		lbvmotCommand->scale = 0.0;	/* clamp it */
	    }
	    lbvmotStatus->feed_scale = lbvmotCommand->scale;
	    break;

	case LBVMOT_RAPID_SCALE:
	    /* override rapids */
	    /* can happen at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "RAPID SCALE");
	    if (lbvmotCommand->scale < 0.0) {
		lbvmotCommand->scale = 0.0;	/* clamp it */
	    }
	    lbvmotStatus->rapid_scale = lbvmotCommand->scale;
	    break;

	case LBVMOT_FS_ENABLE:
	    /* enable/disable overriding speed */
	    /* can happen at any time */
	    if ( lbvmotCommand->mode != 0 ) {
		rtapi_print_msg(RTAPI_MSG_DBG, "FEED SCALE: ON");
		lbvmotStatus->enables_new |= FS_ENABLED;
            } else {
		rtapi_print_msg(RTAPI_MSG_DBG, "FEED SCALE: OFF");
		lbvmotStatus->enables_new &= ~FS_ENABLED;
	    }
	    break;

	case LBVMOT_FH_ENABLE:
	    /* enable/disable feed hold */
	    /* can happen at any time */
	    if ( lbvmotCommand->mode != 0 ) {
		rtapi_print_msg(RTAPI_MSG_DBG, "FEED HOLD: ENABLED");
		lbvmotStatus->enables_new |= FH_ENABLED;
            } else {
		rtapi_print_msg(RTAPI_MSG_DBG, "FEED HOLD: DISABLED");
		lbvmotStatus->enables_new &= ~FH_ENABLED;
	    }
	    break;

	case LBVMOT_SPINDLE_SCALE:
	    /* override spindle speed */
	    /* can happen at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE SCALE");
	    if (lbvmotCommand->scale < 0.0) {
		lbvmotCommand->scale = 0.0;	/* clamp it */
	    }
	    lbvmotStatus->spindle_status[lbvmotCommand->spindle].scale = lbvmotCommand->scale;
	    break;

	case LBVMOT_SS_ENABLE:
	    /* enable/disable overriding spindle speed */
	    /* can happen at any time */
	    if ( lbvmotCommand->mode != 0 ) {
		rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE SCALE: ON");
		lbvmotStatus->enables_new |= SS_ENABLED;
            } else {
		rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE SCALE: OFF");
		lbvmotStatus->enables_new &= ~SS_ENABLED;
	    }
	    break;

	case LBVMOT_AF_ENABLE:
	    /* enable/disable adaptive feedrate override from HAL pin */
	    /* can happen at any time */
	    if ( lbvmotCommand->flags != 0 ) {
		rtapi_print_msg(RTAPI_MSG_DBG, "ADAPTIVE FEED: ON");
		lbvmotStatus->enables_new |= AF_ENABLED;
            } else {
		rtapi_print_msg(RTAPI_MSG_DBG, "ADAPTIVE FEED: OFF");
		lbvmotStatus->enables_new &= ~AF_ENABLED;
	    }
	    break;

	case LBVMOT_DISABLE:
	    /* go into disable */
	    /* can happen at any time */
	    /* reset the lbvmotDebug->enabling flag to defer disable until
	       controller cycle (it *will* be honored) */
	    rtapi_print_msg(RTAPI_MSG_DBG, "DISABLE");
	    lbvmotDebug->enabling = 0;
	    if (lbvmotConfig->kinType == KINEMATICS_INVERSE_ONLY) {
		lbvmotDebug->teleoperating = 0;
		lbvmotDebug->coordinating = 0;
	    }
	    break;

	case LBVMOT_ENABLE:
	    /* come out of disable */
	    /* can happen at any time */
	    /* set the lbvmotDebug->enabling flag to defer enable until
	       controller cycle */
	    rtapi_print_msg(RTAPI_MSG_DBG, "ENABLE");
	    if ( *(lbvmot_hal_data->enable) == 0 ) {
		reportError(_("can't enable motion, enable input is false"));
	    } else {
		lbvmotDebug->enabling = 1;
		if (lbvmotConfig->kinType == KINEMATICS_INVERSE_ONLY) {
		    lbvmotDebug->teleoperating = 0;
		    lbvmotDebug->coordinating = 0;
		}
	    }
	    break;

	case LBVMOT_JOINT_ACTIVATE:
	    /* make joint active, so that amps will be enabled when system is
	       enabled or disabled */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_ACTIVATE");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (joint == 0) {
		break;
	    }
	    SET_JOINT_ACTIVE_FLAG(joint, 1);
	    break;

	case LBVMOT_JOINT_DEACTIVATE:
	    /* make joint inactive, so that amps won't be affected when system
	       is enabled or disabled */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_DEACTIVATE");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (joint == 0) {
		break;
	    }
	    SET_JOINT_ACTIVE_FLAG(joint, 0);
	    break;
	case LBVMOT_JOINT_ENABLE_AMPLIFIER:
	    /* enable the amplifier directly, but don't enable calculations */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_ENABLE_AMP");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (joint == 0) {
		break;
	    }
	    break;

	case LBVMOT_JOINT_DISABLE_AMPLIFIER:
	    /* disable the joint calculations and amplifier, but don't disable
	       calculations */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_DISABLE_AMP");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
	    if (joint == 0) {
		break;
	    }
	    break;

	case LBVMOT_JOINT_HOME:
	    /* home the specified joint */
	    /* need to be in free mode, enable on */
	    /* this just sets the initial state, then the state machine in
	       homing.c does the rest */
	    rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_HOME");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);

	    if (lbvmotStatus->motion_state != LBVMOT_MOTION_FREE) {
		/* can't home unless in free mode */
		reportError(_("must be in joint mode to home"));
		return;
	    }
	    if (*(lbvmot_hal_data->homing_inhibit)) {
	        reportError(_("Homing denied by motion.homing-inhibit joint=%d\n"),
	                   joint_num);
                return;
	    }

	    if (!GET_MOTION_ENABLE_FLAG()) {
		break;
	    }


	    if(joint_num == -1) { // -1 means home all
                if(get_home_sequence_state() == HOME_SEQUENCE_IDLE) {
                    set_home_sequence_state(HOME_SEQUENCE_START);
                } else {
                    reportError(_("homing sequence already in progress"));
                }
	        break;  // do home-all sequence
	    }

	    if (joint == NULL) { break; }
            joint->free_tp.enable = 0; /* abort movement (jog, etc) in progress */

            // ********************************************************
            // support for other homing modes (one sequence, one joint)
            if (get_home_sequence(joint_num) < 0) {
               int jj;
               set_home_sequence_state(HOME_SEQUENCE_DO_ONE_SEQUENCE);
               for (jj = 0; jj < ALL_JOINTS; jj++) {
                  if (ABS(get_home_sequence(jj)) == ABS(get_home_sequence(joint_num))) {
                      // set home_state for all joints at same neg sequence
                      set_home_start(jj);
                  }
               }
               break;
            } else {
               set_home_sequence_state(HOME_SEQUENCE_DO_ONE_JOINT);
               set_home_start(joint_num); //one joint only
            }
	    break;

	case LBVMOT_JOINT_UNHOME:
            /* unhome the specified joint, or all joints if -1 */
            rtapi_print_msg(RTAPI_MSG_DBG, "JOINT_UNHOME");
            rtapi_print_msg(RTAPI_MSG_DBG, " %d", joint_num);
            
            if (   (lbvmotStatus->motion_state != LBVMOT_MOTION_FREE)
                && (lbvmotStatus->motion_state != LBVMOT_MOTION_DISABLED)) {
                reportError(_("must be in joint mode or disabled to unhome"));
                return;
            }

            if (joint_num < 0) {
                /* we want all or none, so these checks need to all be done first.
                 * but, let's only report the first error.  There might be several,
                 * for instance if a homing sequence is running. */
                for (n = 0; n < ALL_JOINTS; n++) {
                    joint = &joints[n];
                    if(GET_JOINT_ACTIVE_FLAG(joint)) {
                        if (get_homing(n)) {
                            reportError(_("Cannot unhome while homing, joint %d"), n);
                            return;
                        }
                        if (!GET_JOINT_INPOS_FLAG(joint)) {
                            reportError(_("Cannot unhome while moving, joint %d"), n);
                            return;
                        }
                    }
                    if (   (n >= NO_OF_KINS_JOINTS)
                        && (lbvmotStatus->motion_state != LBVMOT_MOTION_DISABLED)) {
                        reportError(_("Cannot unhome extrajoint <%d> with motion enabled"), n);
                        return;
                    }
                }
                /* we made it through the checks, so unhome them all */
                for (n = 0; n < ALL_JOINTS; n++) {
                    joint = &joints[n];
                    if(GET_JOINT_ACTIVE_FLAG(joint)) {
                        /* legacy notes:
                        4aa4791cd1 (Chris Radek 2008-02-27 21:07:02 +0000 1310)
                        Unhome support, partly based on a patch by Bryant.
                        Allow unhoming one joint or all (-1) via nml message.
                        A special unhome mode (-2) unhomes only the joints
                        marked as VOLATILE_HOME in the ini.  task could use this
                        to unhome some joints, based on policy, at various state changes.
                        This part is unimplemented so far.
                        */
                        /* if -2, only unhome the volatile_home joints */
                        if( (joint_num != -2) || get_home_is_volatile(n) ) {
                            set_joint_homed(n, 0);
                        }

                    }
                }
            } else if (joint_num < ALL_JOINTS) {
                /* request was for only one joint */
                if (   (joint_num >= NO_OF_KINS_JOINTS)
                    && (lbvmotStatus->motion_state != LBVMOT_MOTION_DISABLED)) {
                    reportError(_("Cannot unhome extrajoint <%d> with motion enabled"), joint_num);
                    return;
                }
                if(GET_JOINT_ACTIVE_FLAG(joint)) {
                    if (get_homing(joint_num) ) {
                        reportError(_("Cannot unhome while homing, joint %d"), joint_num);
                        return;
                    }
                    if (!GET_JOINT_INPOS_FLAG(joint)) {
                        reportError(_("Cannot unhome while moving, joint %d"), joint_num);
                        return;
                    }
                    set_joint_homed(joint_num, 0);
                } else {
                    reportError(_("Cannot unhome inactive joint %d"), joint_num);
                }
            } else {
                /* invalid joint number specified */
                reportError(_("Cannot unhome invalid joint %d (max %d)"), joint_num, (ALL_JOINTS-1));
                return;
            }

            break;

	case LBVMOT_CLEAR_PROBE_FLAGS:
	    rtapi_print_msg(RTAPI_MSG_DBG, "CLEAR_PROBE_FLAGS");
	    lbvmotStatus->probing = 0;
            lbvmotStatus->probeTripped = 0;
	    break;

	case LBVMOT_PROBE:
	    /* most of this is taken from LBVMOT_SET_LINE */
	    /* lbvmotDebug->coord_tp up a linear move */
	    /* requires coordinated mode, enable off, not on limits */
	    rtapi_print_msg(RTAPI_MSG_DBG, "PROBE");
	    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
		reportError(_("need to be enabled, in coord mode for probe move"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!inRange(lbvmotCommand->pos, lbvmotCommand->id, "Probe")) {
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!limits_ok()) {
		reportError(_("can't do probe move with limits exceeded"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!(lbvmotCommand->probe_type & 1)) {
                // if suppress errors = off...

                int probeval = !!*(lbvmot_hal_data->probe_input);
                int probe_whenclears = !!(lbvmotCommand->probe_type & 2);

                if (probeval != probe_whenclears) {
                    // the probe is already in the state we're seeking.
                    if(probe_whenclears) 
                        reportError(_("Probe is already clear when starting G38.4 or G38.5 move"));
                    else
                        reportError(_("Probe is already tripped when starting G38.2 or G38.3 move"));

                    lbvmotStatus->commandStatus = LBVMOT_COMMAND_BAD_EXEC;
                    tpAbort(&lbvmotDebug->coord_tp);
                    SET_MOTION_ERROR_FLAG(1);
                    break;
                }
            }

	    /* append it to the lbvmotDebug->coord_tp */
	    tpSetId(&lbvmotDebug->coord_tp, lbvmotCommand->id);
	    if (-1 == tpAddLine(&lbvmotDebug->coord_tp, lbvmotCommand->pos, lbvmotCommand->motion_type, lbvmotCommand->vel, lbvmotCommand->ini_maxvel, lbvmotCommand->acc, lbvmotStatus->enables_new, 0, -1)) {
		reportError(_("can't add probe move"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_BAD_EXEC;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else {
		lbvmotStatus->probing = 1;
                lbvmotStatus->probe_type = lbvmotCommand->probe_type;
		SET_MOTION_ERROR_FLAG(0);
		/* set flag that indicates all joints need rehoming, if any
		   joint is moved in joint mode, for machines with no forward
		   kins */
		rehomeAll = 1;
	    }
	    break;

	case LBVMOT_RIGID_TAP:
	    /* most of this is taken from LBVMOT_SET_LINE */
	    /* lbvmotDebug->coord_tp up a linear move */
	    /* requires coordinated mode, enable off, not on limits */
	    rtapi_print_msg(RTAPI_MSG_DBG, "RIGID_TAP");
	    if (!GET_MOTION_COORD_FLAG() || !GET_MOTION_ENABLE_FLAG()) {
		reportError(_("need to be enabled, in coord mode for rigid tap move"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!inRange(lbvmotCommand->pos, lbvmotCommand->id, "Rigid tap")) {
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else if (!limits_ok()) {
		reportError(_("can't do rigid tap move with limits exceeded"));
		lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_PARAMS;
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    }

	    /* append it to the lbvmotDebug->coord_tp */
	    tpSetId(&lbvmotDebug->coord_tp, lbvmotCommand->id);
	    int res_addtap = tpAddRigidTap(&lbvmotDebug->coord_tp, lbvmotCommand->pos, lbvmotCommand->vel, lbvmotCommand->ini_maxvel, lbvmotCommand->acc, lbvmotStatus->enables_new, lbvmotCommand->scale);
        if (res_addtap < 0) {
            lbvmotStatus->atspeed_next_feed = 0; /* rigid tap always waits for spindle to be at-speed */
            reportError(_("can't add rigid tap move at line %d, error code %d"),
                    lbvmotCommand->id, res_addtap);
		tpAbort(&lbvmotDebug->coord_tp);
		SET_MOTION_ERROR_FLAG(1);
		break;
	    } else {
		SET_MOTION_ERROR_FLAG(0);
	    }
	    break;

	case LBVMOT_SET_DEBUG:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_DEBUG");
	    lbvmotConfig->debug = lbvmotCommand->debug;
	    lbvmot_config_change();
	    break;

	/* needed for synchronous I/O */
	case LBVMOT_SET_AOUT:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_AOUT");
	    if (lbvmotCommand->now) { //we set it right away
		lbvmotAioWrite(lbvmotCommand->out, lbvmotCommand->minLimit);
	    } else { // we put it on the TP queue, warning: only room for one in there, any new ones will overwrite
		tpSetAout(&lbvmotDebug->coord_tp, lbvmotCommand->out,
		    lbvmotCommand->minLimit, lbvmotCommand->maxLimit);
	    }
	    break;

	case LBVMOT_SET_DOUT:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_DOUT");
	    if (lbvmotCommand->now) { //we set it right away
		lbvmotDioWrite(lbvmotCommand->out, lbvmotCommand->start);
	    } else { // we put it on the TP queue, warning: only room for one in there, any new ones will overwrite
		tpSetDout(&lbvmotDebug->coord_tp, lbvmotCommand->out,
		    lbvmotCommand->start, lbvmotCommand->end);
	    }
	    break;

	case LBVMOT_SPINDLE_ON:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_ON: spindle %d/%d speed %d\n",
                        lbvmotCommand->spindle, lbvmotConfig->numSpindles, (int) lbvmotCommand->vel);
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to start non-existent spindle"));
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    if (*(lbvmot_hal_data->spindle[spindle_num].spindle_orient))
		rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_ORIENT cancelled by SPINDLE_ON\n");
	    if (*(lbvmot_hal_data->spindle[spindle_num].spindle_locked))
		rtapi_print_msg(RTAPI_MSG_DBG, "spindle-locked cleared by SPINDLE_ON\n");
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_locked) = 0;
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_orient) = 0;
	    lbvmotStatus->spindle_status[spindle_num].orient_state = LBVMOT_ORIENT_NONE;

	    /* if (lbvmotStatus->spindle.orient) { */
	    /* 	reportError(_("cant turn on spindle during orient in progress")); */
	    /* 	lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND; */
	    /* 	tpAbort(&lbvmotDebug->tp); */
	    /* 	SET_MOTION_ERROR_FLAG(1); */
	    /* } else {...} */
	    lbvmotStatus->spindle_status[spindle_num].speed = lbvmotCommand->vel;
	    lbvmotStatus->spindle_status[spindle_num].css_factor = lbvmotCommand->ini_maxvel;
	    lbvmotStatus->spindle_status[spindle_num].xoffset = lbvmotCommand->acc;
	    if (lbvmotCommand->vel >= 0) {
		lbvmotStatus->spindle_status[spindle_num].direction = 1;
	    } else {
		lbvmotStatus->spindle_status[spindle_num].direction = -1;
	    }
	    lbvmotStatus->spindle_status[spindle_num].brake = 0; //disengage brake
            lbvmotStatus->atspeed_next_feed = lbvmotCommand->wait_for_spindle_at_speed;

           // check wether it's passed correctly
           if (!lbvmotStatus->atspeed_next_feed)
               rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_ON without wait-for-atspeed");
	    break;

	case LBVMOT_SPINDLE_OFF:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_OFF");
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to stop non-existent spindle <%d>"),spindle_num);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    lbvmotStatus->spindle_status[spindle_num].speed = 0;
	    lbvmotStatus->spindle_status[spindle_num].direction = 0;
	    lbvmotStatus->spindle_status[spindle_num].brake = 1; // engage brake
	    if (*(lbvmot_hal_data->spindle[spindle_num].spindle_orient))
		rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_ORIENT cancelled by SPINDLE_OFF");
	    if (*(lbvmot_hal_data->spindle[spindle_num].spindle_locked))
		rtapi_print_msg(RTAPI_MSG_DBG, "spindle-locked cleared by SPINDLE_OFF");
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_locked) = 0;
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_orient) = 0;
	    lbvmotStatus->spindle_status[spindle_num].orient_state = LBVMOT_ORIENT_NONE;
	    break;

	case LBVMOT_SPINDLE_ORIENT:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_ORIENT");
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to orient non-existent spindle <%d>"),spindle_num);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    if (spindle_num > lbvmotConfig->numSpindles){
            rtapi_print_msg(RTAPI_MSG_ERR, "spindle number <%d> too high in M19",spindle_num);
            break;
	    }
	    if (*(lbvmot_hal_data->spindle[spindle_num].spindle_orient)) {
		rtapi_print_msg(RTAPI_MSG_DBG, "orient already in progress");

		// mah:FIXME unsure wether this is ok or an error
		/* reportError(_("orient already in progress")); */
		/* lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND; */
		/* tpAbort(&lbvmotDebug->tp); */
		/* SET_MOTION_ERROR_FLAG(1); */
	    }
	    lbvmotStatus->spindle_status[spindle_num].orient_state = LBVMOT_ORIENT_IN_PROGRESS;
	    lbvmotStatus->spindle_status[spindle_num].speed = 0;
	    lbvmotStatus->spindle_status[spindle_num].direction = 0;
	    // so far like spindle stop, except opening brake
	    lbvmotStatus->spindle_status[spindle_num].brake = 0; // open brake

	    *(lbvmot_hal_data->spindle[spindle_num].spindle_orient_angle) = lbvmotCommand->orientation;
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_orient_mode) = lbvmotCommand->mode;
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_locked) = 0;
	    *(lbvmot_hal_data->spindle[spindle_num].spindle_orient) = 1;

	    // mirror in spindle status
	    lbvmotStatus->spindle_status[spindle_num].orient_fault = 0; // this pin read during spindle-orient == 1
	    lbvmotStatus->spindle_status[spindle_num].locked = 0;
	    break;

	case LBVMOT_SPINDLE_INCREASE:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_INCREASE");
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to increase non-existent spindle <%d>"),spindle_num);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    if (lbvmotStatus->spindle_status[spindle_num].speed > 0) {
		lbvmotStatus->spindle_status[spindle_num].speed += 100; //FIXME - make the step a HAL parameter
	    } else if (lbvmotStatus->spindle_status[spindle_num].speed < 0) {
		lbvmotStatus->spindle_status[spindle_num].speed -= 100;
	    }
	    break;

	case LBVMOT_SPINDLE_DECREASE:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_DECREASE");
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to decreasenon-existent spindle <%d>"),spindle_num);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    if (lbvmotStatus->spindle_status[spindle_num].speed > 100) {
		lbvmotStatus->spindle_status[spindle_num].speed -= 100; //FIXME - make the step a HAL parameter
	    } else if (lbvmotStatus->spindle_status[spindle_num].speed < -100) {
		lbvmotStatus->spindle_status[spindle_num].speed += 100;
	    }
	    break;

	case LBVMOT_SPINDLE_BRAKE_ENGAGE:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_BRAKE_ENGAGE");
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to engage brake of non-existent spindle <%d>"),spindle_num);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    lbvmotStatus->spindle_status[spindle_num].speed = 0;
	    lbvmotStatus->spindle_status[spindle_num].direction = 0;
	    lbvmotStatus->spindle_status[spindle_num].brake = 1;
	    break;

	case LBVMOT_SPINDLE_BRAKE_RELEASE:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SPINDLE_BRAKE_RELEASE");
	    spindle_num = lbvmotCommand->spindle;
        if (spindle_num >= lbvmotConfig->numSpindles){
            reportError(_("Attempt to release brake of non-existent spindle <%d>"),spindle_num);
            lbvmotStatus->commandStatus = LBVMOT_COMMAND_INVALID_COMMAND;
            break;
        }
	    lbvmotStatus->spindle_status[spindle_num].brake = 0;
	    break;

	case LBVMOT_SET_JOINT_COMP:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_JOINT_COMP for joint %d", joint_num);
	    if (joint == 0) {
		break;
	    }
	    if (joint->comp.entries >= LBVMOT_COMP_SIZE) {
		reportError(_("joint %d: too many compensation entries"), joint_num);
		break;
	    }
	    /* point to last entry */
	    comp_entry = &(joint->comp.array[joint->comp.entries]);
	    if (lbvmotCommand->comp_nominal <= comp_entry[0].nominal) {
		reportError(_("joint %d: compensation values must increase"), joint_num);
		break;
	    }
	    /* store data to new entry */
	    comp_entry[1].nominal = lbvmotCommand->comp_nominal;
	    comp_entry[1].fwd_trim = lbvmotCommand->comp_forward;
	    comp_entry[1].rev_trim = lbvmotCommand->comp_reverse;
	    /* calculate slopes from previous entry to the new one */
	    if ( comp_entry[0].nominal != -DBL_MAX ) {
		/* but only if the previous entry is "real" */
		tmp1 = comp_entry[1].nominal - comp_entry[0].nominal;
		comp_entry[0].fwd_slope =
		    (comp_entry[1].fwd_trim - comp_entry[0].fwd_trim) / tmp1;
		comp_entry[0].rev_slope =
		    (comp_entry[1].rev_trim - comp_entry[0].rev_trim) / tmp1;
	    } else {
		/* previous entry is at minus infinity, slopes are zero */
		comp_entry[0].fwd_trim = comp_entry[1].fwd_trim;
		comp_entry[0].rev_trim = comp_entry[1].rev_trim;
	    }
	    joint->comp.entries++;
	    break;

        case LBVMOT_SET_OFFSET:
            lbvmotStatus->tool_offset = lbvmotCommand->tool_offset;
            break;

	case LBVMOT_SET_AXIS_POSITION_LIMITS:
	    /* set the position limits for axis */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_AXIS_POSITION_LIMITS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", axis_num);
	    lbvmot_config_change();
	    if (axis == 0) {
		break;
	    }
	    axis->min_pos_limit = lbvmotCommand->minLimit;
	    axis->max_pos_limit = lbvmotCommand->maxLimit;
	    break;

        case LBVMOT_SET_AXIS_VEL_LIMIT:
	    /* set the max axis vel */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_AXIS_VEL_LIMITS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", axis_num);
	    lbvmot_config_change();
	    if (axis == 0) {
		break;
	    }
	    axis->vel_limit = lbvmotCommand->vel;
	    axis->ext_offset_vel_limit = lbvmotCommand->ext_offset_vel;
            break;

        case LBVMOT_SET_AXIS_ACC_LIMIT:
 	    /* set the max axis acc */
	    /* can be done at any time */
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_AXIS_ACC_LIMITS");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", axis_num);
	    lbvmot_config_change();
	    if (axis == 0) {
		break;
	    }
	    axis->acc_limit = lbvmotCommand->acc;
	    axis->ext_offset_acc_limit = lbvmotCommand->ext_offset_acc;
            break;

        case LBVMOT_SET_AXIS_LOCKING_JOINT:
	    rtapi_print_msg(RTAPI_MSG_DBG, "SET_AXIS_ACC_LOCKING_JOINT");
	    rtapi_print_msg(RTAPI_MSG_DBG, " %d", axis_num);
	    lbvmot_config_change();
	    if (axis == 0) {
		break;
	    }
	    axis->locking_joint = joint_num;
            break;

	default:
	    rtapi_print_msg(RTAPI_MSG_DBG, "UNKNOWN");
	    reportError(_("unrecognized command %d"), lbvmotCommand->command);
	    lbvmotStatus->commandStatus = LBVMOT_COMMAND_UNKNOWN_COMMAND;
	    break;
        case LBVMOT_SET_MAX_FEED_OVERRIDE:
            lbvmotConfig->maxFeedScale = lbvmotCommand->maxFeedScale;
            break;
        case LBVMOT_SETUP_ARC_BLENDS:
            lbvmotConfig->arcBlendEnable = lbvmotCommand->arcBlendEnable;
            lbvmotConfig->arcBlendFallbackEnable = lbvmotCommand->arcBlendFallbackEnable;
            lbvmotConfig->arcBlendOptDepth = lbvmotCommand->arcBlendOptDepth;
            lbvmotConfig->arcBlendGapCycles = lbvmotCommand->arcBlendGapCycles;
            lbvmotConfig->arcBlendRampFreq = lbvmotCommand->arcBlendRampFreq;
            lbvmotConfig->arcBlendTangentKinkRatio = lbvmotCommand->arcBlendTangentKinkRatio;
            break;
        case LBVMOT_SET_PROBE_ERR_INHIBIT:
            lbvmotConfig->inhibit_probe_jog_error = lbvmotCommand->probe_jog_err_inhibit;
            lbvmotConfig->inhibit_probe_home_error = lbvmotCommand->probe_home_err_inhibit;
            break;

	}			/* end of: command switch */
	if (lbvmotStatus->commandStatus != LBVMOT_COMMAND_OK) {
	    rtapi_print_msg(RTAPI_MSG_DBG, "ERROR: %d",
		lbvmotStatus->commandStatus);
	}
	rtapi_print_msg(RTAPI_MSG_DBG, "\n");
	/* synch tail count */
	lbvmotStatus->tail = lbvmotStatus->head;
	lbvmotConfig->tail = lbvmotConfig->head;
	lbvmotDebug->tail = lbvmotDebug->head;

    }
    /* end of: if-new-command */

    return;
}
