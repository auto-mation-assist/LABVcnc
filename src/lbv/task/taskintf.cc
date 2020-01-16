/********************************************************************
* Description: taskintf.cc
*   Interface functions for motion.
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/

#include <cmath>
#include <float.h>		// DBL_MAX
#include <string.h>		// memcpy() strncpy()
#include <unistd.h>             // unlink()

#include "usrmotintf.h"		// usrmotInit(), usrmotReadLbvmotStatus(),
				// etc.
#include "motion.h"		// lbvmot_command_t,STATUS, etc.
#include "motion_debug.h"
#include "lbv.hh"
#include "lbvcfg.h"		// LBV_INIFILE
#include "lbvglb.h"		// LBV_INIFILE
#include "lbv_nml.hh"
#include "rcs_print.hh"
#include "timer.hh"
#include "inifile.hh"
#include "iniaxis.hh"
#include "inijoint.hh"
#include "initraj.hh"
#include "inihal.hh"

value_inihal_data old_inihal_data;

/* define this to catch isnan errors, for rtlinux FPU register 
   problem testing */
#define ISNAN_TRAP

#ifdef ISNAN_TRAP
#define CATCH_NAN(cond) do {                           \
    if (cond) {                                        \
        printf("isnan error in %s()\n", __FUNCTION__); \
        return -1;                                     \
    }                                                  \
} while(0)
#else
#define CATCH_NAN(cond) do {} while(0)
#endif


// MOTION INTERFACE

/*! \todo FIXME - this decl was originally much later in the file, moved
here temporarily for debugging */
static lbvmot_status_t lbvmotStatus;

/*
  Implementation notes:

  Initing:  the lbvmot interface needs to be inited once, but nml_traj_init()
  and nml_servo_init() can be called in any order. Similarly, the lbvmot
  interface needs to be exited once, but nml_traj_exit() and nml_servo_exit()
  can be called in any order. They can also be called multiple times. Flags
  are used to signify if initing has been done, or if the final exit has
  been called.
  */

static struct TrajConfig_t TrajConfig;
static struct JointConfig_t JointConfig[LBVMOT_MAX_JOINTS];
static struct AxisConfig_t AxisConfig[LBVMOT_MAX_AXIS];

static lbvmot_command_t lbvmotCommand;

__attribute__ ((unused))
static int lbvmotIoInited = 0;	// non-zero means io called init
static int lbvmotion_initialized = 0;	// non-zero means both
						// lbvMotionInit called.

// local status data, not provided by lbvmot
static unsigned long localMotionHeartbeat = 0;
static int localMotionCommandType = 0;
static int localMotionEchoSerialNumber = 0;

//FIXME-AJ: see if needed
//static double localLbvAxisUnits[LBVMOT_MAX_AXIS];

// axes and joints are numbered 0..NUM-1

/*
  In lbvmot, we need to set the cycle time for traj, and the interpolation
  rate, in any order, but both need to be done. 
 */

/*! functions involving joints */

int lbvJointSetType(int joint, unsigned char jointType)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].Type = jointType;

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %d)\n", __FUNCTION__, joint, jointType);
    }
    return 0;
}

int lbvJointSetUnits(int joint, double units)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].Units = units;

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f)\n", __FUNCTION__, joint, units);
    }
    return 0;
}

int lbvJointSetBacklash(int joint, double backlash)
{
#ifdef ISNAN_TRAP
    if (std::isnan(backlash)) {
	printf("std::isnan error in lbvJointSetBacklash()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_SET_JOINT_BACKLASH;
    lbvmotCommand.joint = joint;
    lbvmotCommand.backlash = backlash;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, backlash, retval);
    }
    return retval;
}

int lbvJointSetMinPositionLimit(int joint, double limit)
{
#ifdef ISNAN_TRAP
    if (std::isnan(limit)) {
	printf("isnan error in lbvJointSetMinPosition()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].MinLimit = limit;

    lbvmotCommand.command = LBVMOT_SET_JOINT_POSITION_LIMITS;
    lbvmotCommand.joint = joint;
    lbvmotCommand.minLimit = JointConfig[joint].MinLimit;
    lbvmotCommand.maxLimit = JointConfig[joint].MaxLimit;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, limit, retval);
    }
    return retval;
}

int lbvJointSetMaxPositionLimit(int joint, double limit)
{
#ifdef ISNAN_TRAP
    if (std::isnan(limit)) {
	printf("std::isnan error in lbvJointSetMaxPosition()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    JointConfig[joint].MaxLimit = limit;

    lbvmotCommand.command = LBVMOT_SET_JOINT_POSITION_LIMITS;
    lbvmotCommand.joint = joint;
    lbvmotCommand.minLimit = JointConfig[joint].MinLimit;
    lbvmotCommand.maxLimit = JointConfig[joint].MaxLimit;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, limit, retval);
    }
    return retval;
}

int lbvJointSetMotorOffset(int joint, double offset) 
{
#ifdef ISNAN_TRAP
    if (std::isnan(offset)) {
	printf("isnan error in lbvJointSetMotorOffset()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }
    lbvmotCommand.command = LBVMOT_SET_JOINT_MOTOR_OFFSET;
    lbvmotCommand.joint = joint;
    lbvmotCommand.motor_offset = offset;
    
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, offset, retval);
    }
    return retval;
}

int lbvJointSetFerror(int joint, double ferror)
{
#ifdef ISNAN_TRAP
    if (std::isnan(ferror)) {
	printf("isnan error in lbvJointSetFerror()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_SET_JOINT_MAX_FERROR;
    lbvmotCommand.joint = joint;
    lbvmotCommand.maxFerror = ferror;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, ferror, retval);
    }
    return retval;
}

int lbvJointSetMinFerror(int joint, double ferror)
{
#ifdef ISNAN_TRAP
    if (std::isnan(ferror)) {
	printf("isnan error in lbvJointSetMinFerror()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }
    lbvmotCommand.command = LBVMOT_SET_JOINT_MIN_FERROR;
    lbvmotCommand.joint = joint;
    lbvmotCommand.minFerror = ferror;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, ferror, retval);
    }
    return retval;
}

int lbvJointSetHomingParams(int joint, double home, double offset, double home_final_vel,
			   double search_vel, double latch_vel,
			   int use_index, int encoder_does_not_reset,
			   int ignore_limits, int is_shared,
			   int sequence,int volatile_home, int locking_indexer,int absolute_encoder)
{
#ifdef ISNAN_TRAP
    if (std::isnan(home) || std::isnan(offset) || std::isnan(home_final_vel) ||
	std::isnan(search_vel) || std::isnan(latch_vel)) {
	printf("isnan error in lbvJointSetHomingParams()\n");
	return -1;
    }
#endif

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_SET_JOINT_HOMING_PARAMS;
    lbvmotCommand.joint = joint;
    lbvmotCommand.home = home;
    lbvmotCommand.offset = offset;
    lbvmotCommand.home_final_vel = home_final_vel;
    lbvmotCommand.search_vel = search_vel;
    lbvmotCommand.latch_vel = latch_vel;
    lbvmotCommand.flags = 0;
    lbvmotCommand.home_sequence = sequence;
    lbvmotCommand.volatile_home = volatile_home;
    if (use_index) {
	lbvmotCommand.flags |= HOME_USE_INDEX;
    }
    if (encoder_does_not_reset) {
	lbvmotCommand.flags |= HOME_INDEX_NO_ENCODER_RESET;
    }
    if (ignore_limits) {
	lbvmotCommand.flags |= HOME_IGNORE_LIMITS;
    }
    if (is_shared) {
	lbvmotCommand.flags |= HOME_IS_SHARED;
    }
    if (locking_indexer) {
        lbvmotCommand.flags |= HOME_UNLOCK_FIRST;
    }
    if (absolute_encoder) {
        switch (absolute_encoder) {
          case 0: break;
          case 1: lbvmotCommand.flags |= HOME_ABSOLUTE_ENCODER;
                  lbvmotCommand.flags |= HOME_NO_REHOME;
                  break;
          case 2: lbvmotCommand.flags |= HOME_ABSOLUTE_ENCODER;
                  lbvmotCommand.flags |= HOME_NO_REHOME;
                  lbvmotCommand.flags |= HOME_NO_FINAL_MOVE;
                  break;
          default: fprintf(stderr,
                   "Unknown option for absolute_encoder <%d>",absolute_encoder);
                  break;
        }
    }

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f, %.4f, %.4f, %.4f, %.4f, %d, %d, %d, %d, %d) returned %d\n",
          __FUNCTION__, joint, home, offset, home_final_vel, search_vel, latch_vel,
          use_index, ignore_limits, is_shared, sequence, volatile_home, retval);
    }
    return retval;
}

int lbvJointUpdateHomingParams(int joint, double home, double offset, int sequence)
{
    CATCH_NAN(std::isnan(home) || std::isnan(offset) );

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_UPDATE_JOINT_HOMING_PARAMS;
    lbvmotCommand.joint = joint;
    lbvmotCommand.home = home;
    lbvmotCommand.offset = offset;
    lbvmotCommand.home_sequence = sequence;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f, %.4f) returned %d\n",
          __FUNCTION__, joint, home, offset,retval);
    }
    return retval;
}

int lbvJointSetMaxVelocity(int joint, double vel)
{
    CATCH_NAN(std::isnan(vel));

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    if (vel < 0.0) {
	vel = 0.0;
    }

    JointConfig[joint].MaxVel = vel;

    lbvmotCommand.command = LBVMOT_SET_JOINT_VEL_LIMIT;
    lbvmotCommand.joint = joint;
    lbvmotCommand.vel = vel;
    
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, vel, retval);
    }
    return retval;
}

int lbvJointSetMaxAcceleration(int joint, double acc)
{
    CATCH_NAN(std::isnan(acc));

    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }
    if (acc < 0.0) {
	acc = 0.0;
    }
    JointConfig[joint].MaxAccel = acc;
    //FIXME-AJ: need functions for setting the AXIS_MAX_ACCEL (either from the ini, or from kins..)
    lbvmotCommand.command = LBVMOT_SET_JOINT_ACC_LIMIT;
    lbvmotCommand.joint = joint;
    lbvmotCommand.acc = acc;
    
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, joint, acc, retval);
    }
    return retval;
}

/*! functions involving carthesian Axes (X,Y,Z,A,B,C,U,V,W) */
    
int lbvAxisSetMinPositionLimit(int axis, double limit)
{
    CATCH_NAN(std::isnan(limit));

    if (axis < 0 || axis >= LBVMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    AxisConfig[axis].MinLimit = limit;

    lbvmotCommand.command = LBVMOT_SET_AXIS_POSITION_LIMITS;
    lbvmotCommand.axis = axis;
    lbvmotCommand.minLimit = AxisConfig[axis].MinLimit;
    lbvmotCommand.maxLimit = AxisConfig[axis].MaxLimit;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, limit, retval);
    }
    return retval;
}

int lbvAxisSetMaxPositionLimit(int axis, double limit)
{
    CATCH_NAN(std::isnan(limit));

    if (axis < 0 || axis >= LBVMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    AxisConfig[axis].MaxLimit = limit;

    lbvmotCommand.command = LBVMOT_SET_AXIS_POSITION_LIMITS;
    lbvmotCommand.axis = axis;
    lbvmotCommand.minLimit = AxisConfig[axis].MinLimit;
    lbvmotCommand.maxLimit = AxisConfig[axis].MaxLimit;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, limit, retval);
    }
    return retval;
}

int lbvAxisSetMaxVelocity(int axis, double vel,double ext_offset_vel)
{
    CATCH_NAN(std::isnan(vel));

    if (axis < 0 || axis >= LBVMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    if (vel < 0.0) {
	vel = 0.0;
    }

    AxisConfig[axis].MaxVel = vel;

    lbvmotCommand.command = LBVMOT_SET_AXIS_VEL_LIMIT;
    lbvmotCommand.axis = axis;
    lbvmotCommand.vel = vel;
    lbvmotCommand.ext_offset_vel = ext_offset_vel;
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, vel, retval);
    }
    return retval;
}

int lbvAxisSetMaxAcceleration(int axis, double acc,double ext_offset_acc)
{
    CATCH_NAN(std::isnan(acc));

    if (axis < 0 || axis >= LBVMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    if (acc < 0.0) {
	acc = 0.0;
    }
    
    AxisConfig[axis].MaxAccel = acc;    

    lbvmotCommand.command = LBVMOT_SET_AXIS_ACC_LIMIT;
    lbvmotCommand.axis = axis;
    lbvmotCommand.acc = acc;
    lbvmotCommand.ext_offset_acc = ext_offset_acc;
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %.4f) returned %d\n", __FUNCTION__, axis, acc, retval);
    }
    return retval;
}

int lbvAxisSetLockingJoint(int axis, int joint)
{

    if (axis < 0 || axis >= LBVMOT_MAX_AXIS || !(TrajConfig.AxisMask & (1 << axis))) {
	return 0;
    }

    if (joint < 0) {
	joint = -1;
    }

    lbvmotCommand.command = LBVMOT_SET_AXIS_LOCKING_JOINT;
    lbvmotCommand.axis    = axis;
    lbvmotCommand.joint   = joint;
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %d) returned %d\n", __FUNCTION__, axis, joint, retval);
    }
    return retval;
}

double lbvAxisGetMaxVelocity(int axis)
{
    if (axis < 0 || axis >= LBVMOT_MAX_AXIS) {
        return 0;
    }

    return AxisConfig[axis].MaxVel;
}

double lbvAxisGetMaxAcceleration(int axis)
{
    if (axis < 0 || axis >= LBVMOT_MAX_AXIS) {
        return 0;
    }

    return AxisConfig[axis].MaxAccel;
}

int lbvAxisUpdate(LBV_AXIS_STAT stat[], int axis_mask)
{
    int axis_num;
    lbvmot_axis_status_t *axis;
    
    for (axis_num = 0; axis_num < LBVMOT_MAX_AXIS; axis_num++) {
        if(!(axis_mask & (1 << axis_num))) continue;
        axis = &(lbvmotStatus.axis_status[axis_num]);

        stat[axis_num].velocity = axis->teleop_vel_cmd;
        stat[axis_num].minPositionLimit = axis->min_pos_limit;
        stat[axis_num].maxPositionLimit = axis->max_pos_limit;
    }
    return 0;
}

/* This function checks to see if any joint or the traj has
   been inited already.  At startup, if none have been inited,
   usrmotIniLoad and usrmotInit must be called first.  At
   shutdown, after all have been halted, the usrmotExit must
   be called.
*/

static int JointOrTrajInited(void)
{
    int joint;

    for (joint = 0; joint < LBVMOT_MAX_JOINTS; joint++) {
	if (JointConfig[joint].Inited) {
	    return 1;
	}
    }
    if (TrajConfig.Inited) {
	return 1;
    }
    return 0;
}

int lbvJointInit(int joint)
{
    int retval = 0;
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }
    // init lbvmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(lbv_inifile);
	if (0 != usrmotInit("lbv2_task")) {
	    return -1;
	}
    }
    JointConfig[joint].Inited = 1;
    if (0 != iniJoint(joint, lbv_inifile)) {
	retval = -1;
    }
    return retval;
}

int lbvAxisInit(int axis)
{
    int retval = 0;

    if (axis < 0 || axis >= LBVMOT_MAX_AXIS) {
	return 0;
    }
    // init lbvmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(lbv_inifile);
	if (0 != usrmotInit("lbv2_task")) {
	    return -1;
	}
    }
    AxisConfig[axis].Inited = 1;
    if (0 != iniAxis(axis, lbv_inifile)) {
	retval = -1;
    }
    return retval;
}

int lbvJointHalt(int joint)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }
    /*! \todo FIXME-- refs global lbvStatus; should make LBV_JOINT_STAT an arg here */
    if (NULL != lbvStatus && lbvmotion_initialized
	&& JointConfig[joint].Inited) {
	//dumpJoint(joint, lbv_inifile, &lbvStatus->motion.joint[joint]);
    }
    JointConfig[joint].Inited = 0;

    if (!JointOrTrajInited()) {
	usrmotExit();		// ours is final exit
    }

    return 0;
}

int lbvJointAbort(int joint)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }
    lbvmotCommand.command = LBVMOT_JOINT_ABORT;
    lbvmotCommand.joint = joint;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJointActivate(int joint)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_JOINT_ACTIVATE;
    lbvmotCommand.joint = joint;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d) returned %d\n", __FUNCTION__, joint, retval);
    }
    return retval;
}

int lbvJointDeactivate(int joint)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_JOINT_DEACTIVATE;
    lbvmotCommand.joint = joint;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJointOverrideLimits(int joint)
{
    // can have joint < 0, for resuming normal limit checking
    if (joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_OVERRIDE_LIMITS;
    lbvmotCommand.joint = joint;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJointEnable(int joint)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_JOINT_ENABLE_AMPLIFIER;
    lbvmotCommand.joint = joint;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJointDisable(int joint)
{
    if (joint < 0 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_JOINT_DISABLE_AMPLIFIER;
    lbvmotCommand.joint = joint;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJointHome(int joint)
{
    if (joint < -1 || joint >= LBVMOT_MAX_JOINTS) {
	return 0;
    }

    lbvmotCommand.command = LBVMOT_JOINT_HOME;
    lbvmotCommand.joint = joint;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJointUnhome(int joint)
{
	if (joint < -2 || joint >= LBVMOT_MAX_JOINTS) {
		return 0;
	}

	lbvmotCommand.command = LBVMOT_JOINT_UNHOME;
	lbvmotCommand.joint = joint;

	return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJogCont(int nr, double vel, int jjogmode)
{
    if (jjogmode) {
        if (nr < 0 || nr >= LBVMOT_MAX_JOINTS) { return 0; }
        if (vel > JointConfig[nr].MaxVel) {
            vel = JointConfig[nr].MaxVel;
        } else if (vel < -JointConfig[nr].MaxVel) {
            vel = -JointConfig[nr].MaxVel;
        }
        lbvmotCommand.joint = nr;
        lbvmotCommand.axis = -1;  //NA
    } else {
        if (nr < 0 || nr >= LBVMOT_MAX_AXIS) { return 0; }
        if (vel > AxisConfig[nr].MaxVel) {
            vel = AxisConfig[nr].MaxVel;
        } else if (vel < -AxisConfig[nr].MaxVel) {
            vel = -AxisConfig[nr].MaxVel;
        }
        lbvmotCommand.joint = -1; //NA
        lbvmotCommand.axis = nr;
    }
    lbvmotCommand.command = LBVMOT_JOG_CONT;
    lbvmotCommand.vel = vel;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJogIncr(int nr, double incr, double vel, int jjogmode)
{
    if (jjogmode) {
        if (nr < 0 || nr >= LBVMOT_MAX_JOINTS) { return 0; }
        if (vel > JointConfig[nr].MaxVel) {
            vel = JointConfig[nr].MaxVel;
        } else if (vel < -JointConfig[nr].MaxVel) {
            vel = -JointConfig[nr].MaxVel;
        }
        lbvmotCommand.joint = nr;
        lbvmotCommand.axis = -1; //NA
    } else {
        if (nr < 0 || nr >= LBVMOT_MAX_AXIS) { return 0; }
        if (vel > AxisConfig[nr].MaxVel) {
            vel = AxisConfig[nr].MaxVel;
        } else if (vel < -AxisConfig[nr].MaxVel) {
            vel = -AxisConfig[nr].MaxVel;
        }
        lbvmotCommand.joint = -1; //NA
        lbvmotCommand.axis = nr;
    }
    lbvmotCommand.command = LBVMOT_JOG_INCR;
    lbvmotCommand.vel = vel;
    lbvmotCommand.offset = incr;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJogAbs(int nr, double pos, double vel, int jjogmode)
{
    if (jjogmode) {        
        if (nr < 0 || nr >= LBVMOT_MAX_JOINTS) { return 0; }
        if (vel > JointConfig[nr].MaxVel) {
            vel = JointConfig[nr].MaxVel;
        } else if (vel < -JointConfig[nr].MaxVel) {
            vel = -JointConfig[nr].MaxVel;
        }
        lbvmotCommand.joint = nr;
        lbvmotCommand.axis = -1; //NA
    } else {
        if (nr < 0 || nr >= LBVMOT_MAX_AXIS) { return 0; }
        if (vel > AxisConfig[nr].MaxVel) {
            vel = AxisConfig[nr].MaxVel;
        } else if (vel < -AxisConfig[nr].MaxVel) {
            vel = -AxisConfig[nr].MaxVel;
        }
        lbvmotCommand.joint = -1; //NA
        lbvmotCommand.axis = nr;
    }
    lbvmotCommand.command = LBVMOT_JOG_ABS;
    lbvmotCommand.vel = vel;
    lbvmotCommand.offset = pos;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvJogStop(int nr, int jjogmode)
{
    if (jjogmode) {
        if (nr < 0 || nr >= LBVMOT_MAX_JOINTS) { return 0; }
        lbvmotCommand.joint = nr;
        lbvmotCommand.axis = -1; //NA
    } else {
        if (nr < 0 || nr >= LBVMOT_MAX_AXIS) { return 0; }
        lbvmotCommand.joint = -1; //NA
        lbvmotCommand.axis = nr;
    }
    lbvmotCommand.command = LBVMOT_JOINT_ABORT;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}


int lbvJointLoadComp(int joint, const char *file, int type)
{
    return usrmotLoadComp(joint, file, type);
}

static lbvmot_config_t lbvmotConfig;
int get_lbvmot_debug_info = 0;

/*
  these globals are set in lbvMotionUpdate(), then referenced in
  lbvJointUpdate(), lbvTrajUpdate() to save calls to usrmotReadLbvmotStatus
 */
static lbvmot_debug_t lbvmotDebug;
static char errorString[LBVMOT_ERROR_LEN];
static int new_config = 0;

/*! \todo FIXME - debugging - uncomment the following line to log changes in
   JOINT_FLAG */
// #define WATCH_FLAGS 1

int lbvJointUpdate(LBV_JOINT_STAT stat[], int numJoints)
{
/*! \todo FIXME - this function accesses data that has been
   moved.  Once I know what it is used for I'll fix it */

    int joint_num;
    lbvmot_joint_status_t *joint;
#ifdef WATCH_FLAGS
    static int old_joint_flag[8];
#endif

    // check for valid range
    if (numJoints <= 0 || numJoints > LBVMOT_MAX_JOINTS) {
	return -1;
    }

    for (joint_num = 0; joint_num < numJoints; joint_num++) {
	/* point to joint data */

	joint = &(lbvmotStatus.joint_status[joint_num]);

	stat[joint_num].jointType = JointConfig[joint_num].Type;
	stat[joint_num].units = JointConfig[joint_num].Units;
	if (new_config) {
	    stat[joint_num].backlash = joint->backlash;
	    stat[joint_num].minPositionLimit = joint->min_pos_limit;
	    stat[joint_num].maxPositionLimit = joint->max_pos_limit;
	    stat[joint_num].minFerror = joint->min_ferror;
	    stat[joint_num].maxFerror = joint->max_ferror;
/*! \todo FIXME - should all homing config params be included here? */
//	    stat[joint_num].homeOffset = joint->home_offset;
	}
	stat[joint_num].output = joint->pos_cmd;
	stat[joint_num].input = joint->pos_fb;
	stat[joint_num].velocity = joint->vel_cmd;
	stat[joint_num].ferrorCurrent = joint->ferror;
	stat[joint_num].ferrorHighMark = joint->ferror_high_mark;

	stat[joint_num].homing = joint->homing;
	stat[joint_num].homed  = joint->homed;

	stat[joint_num].fault = (joint->flag & LBVMOT_JOINT_FAULT_BIT ? 1 : 0);
	stat[joint_num].enabled = (joint->flag & LBVMOT_JOINT_ENABLE_BIT ? 1 : 0);
	stat[joint_num].inpos = (joint->flag & LBVMOT_JOINT_INPOS_BIT ? 1 : 0);

/* FIXME - soft limits are now applied to the command, and should never
   happen */
	stat[joint_num].minSoftLimit = 0;
	stat[joint_num].maxSoftLimit = 0;
	stat[joint_num].minHardLimit =
	    (joint->flag & LBVMOT_JOINT_MIN_HARD_LIMIT_BIT ? 1 : 0);
	stat[joint_num].maxHardLimit =
	    (joint->flag & LBVMOT_JOINT_MAX_HARD_LIMIT_BIT ? 1 : 0);
	stat[joint_num].overrideLimits = !!(lbvmotStatus.overrideLimitMask);	// one
	// for
	// all

#ifdef WATCH_FLAGS
	if (old_joint_flag[joint_num] != joint->flag) {
	    printf("joint %d flag: %04X -> %04X\n", joint_num,
		   old_joint_flag[joint_num], joint->flag);
	    old_joint_flag[joint_num] = joint->flag;
	}
#endif
	if (joint->flag & LBVMOT_JOINT_ERROR_BIT) {
	    if (stat[joint_num].status != RCS_ERROR) {
		rcs_print_error("Error on joint %d, command number %d\n",
				joint_num, lbvmotStatus.commandNumEcho);
		stat[joint_num].status = RCS_ERROR;
	    }
	} else if (joint->flag & LBVMOT_JOINT_INPOS_BIT) {
	    stat[joint_num].status = RCS_DONE;
	} else {
	    stat[joint_num].status = RCS_EXEC;
	}
    }
    return 0;
}

// LBV_TRAJ functions

int lbvTrajSetJoints(int joints)
{
    if (joints <= 0 || joints > LBVMOT_MAX_JOINTS) {
	rcs_print("lbvTrajSetJoints failing: joints=%d\n",
		joints);
	return -1;
    }

    TrajConfig.Joints = joints;
    lbvmotCommand.command = LBVMOT_SET_NUM_JOINTS;
    lbvmotCommand.joint = joints;
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d) returned %d\n", __FUNCTION__, joints, retval);
    }
    return retval;
}

int lbvTrajSetAxes(int axismask)
{
    int axes = 0;
    for(int i=0; i<LBVMOT_MAX_AXIS; i++)
        if(axismask & (1<<i)) axes = i+1;

    TrajConfig.DeprecatedAxes = axes;
    TrajConfig.AxisMask = axismask;
    
    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d, %d)\n", __FUNCTION__, axes, axismask);
    }
    return 0;
}

int lbvTrajSetSpindles(int spindles)
{
    if (spindles <= 0 || spindles > LBVMOT_MAX_SPINDLES) {
	rcs_print("lbvTrajSetSpindles failing: spindles=%d\n",
		spindles);
	return -1;
    }

    TrajConfig.Spindles = spindles;
    lbvmotCommand.command = LBVMOT_SET_NUM_SPINDLES;
    lbvmotCommand.spindle = spindles;
    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%d) returned %d\n", __FUNCTION__, spindles, retval);
    }
    return retval;
}

int lbvTrajSetUnits(double linearUnits, double angularUnits)
{
    if (linearUnits <= 0.0 || angularUnits <= 0.0) {
	return -1;
    }

    TrajConfig.LinearUnits = linearUnits;
    TrajConfig.AngularUnits = angularUnits;

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%.4f, %.4f)\n", __FUNCTION__, linearUnits, angularUnits);
    }
    return 0;
}

int lbvTrajSetMode(int mode)
{
    switch (mode) {
    case LBV_TRAJ_MODE_FREE:
	lbvmotCommand.command = LBVMOT_FREE;
	return usrmotWriteLbvmotCommand(&lbvmotCommand);

    case LBV_TRAJ_MODE_COORD:
	lbvmotCommand.command = LBVMOT_COORD;
	return usrmotWriteLbvmotCommand(&lbvmotCommand);

    case LBV_TRAJ_MODE_TELEOP:
	lbvmotCommand.command = LBVMOT_TELEOP;
	return usrmotWriteLbvmotCommand(&lbvmotCommand);

    default:
	return -1;
    }
}

int lbvTrajSetVelocity(double vel, double ini_maxvel)
{
    if (vel < 0.0) {
	vel = 0.0;
    } else if (vel > TrajConfig.MaxVel) {
	vel = TrajConfig.MaxVel;
    }

    if (ini_maxvel < 0.0) {
	    ini_maxvel = 0.0;
    } else if (vel > TrajConfig.MaxVel) {
	    ini_maxvel = TrajConfig.MaxVel;
    }

    lbvmotCommand.command = LBVMOT_SET_VEL;
    lbvmotCommand.vel = vel;
    lbvmotCommand.ini_maxvel = ini_maxvel;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%.4f, %.4f) returned %d\n", __FUNCTION__, vel, ini_maxvel, retval);
    }
    return retval;
}

int lbvTrajSetAcceleration(double acc)
{
    if (acc < 0.0) {
	acc = 0.0;
    } else if (acc > TrajConfig.MaxAccel) {
	acc = TrajConfig.MaxAccel;
    }

    lbvmotCommand.command = LBVMOT_SET_ACC;
    lbvmotCommand.acc = acc;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%.4f) returned %d\n", __FUNCTION__, acc, retval);
    }
    return retval;
}

/*
  lbvmot has no limits on max velocity, acceleration so we'll save them
  here and apply them in the functions above
  */
int lbvTrajSetMaxVelocity(double vel)
{
    if (vel < 0.0) {
	vel = 0.0;
    }

    TrajConfig.MaxVel = vel;

    lbvmotCommand.command = LBVMOT_SET_VEL_LIMIT;
    lbvmotCommand.vel = vel;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%.4f) returned %d\n", __FUNCTION__, vel, retval);
    }
    return retval;
}

int lbvTrajSetMaxAcceleration(double acc)
{
    if (acc < 0.0) {
	acc = 0.0;
    }

    TrajConfig.MaxAccel = acc;

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%.4f)\n", __FUNCTION__, acc);
    }
    return 0;
}

int lbvTrajSetHome(LbvPose home)
{
#ifdef ISNAN_TRAP
    if (std::isnan(home.tran.x) || std::isnan(home.tran.y) || std::isnan(home.tran.z) ||
	std::isnan(home.a) || std::isnan(home.b) || std::isnan(home.c) ||
	std::isnan(home.u) || std::isnan(home.v) || std::isnan(home.w)) {
	printf("std::isnan error in lbvTrajSetHome()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    lbvmotCommand.command = LBVMOT_SET_WORLD_HOME;
    lbvmotCommand.pos = home;

    int retval = usrmotWriteLbvmotCommand(&lbvmotCommand);

    if (lbv_debug & LBV_DEBUG_CONFIG) {
        rcs_print("%s(%.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f, %.4f) returned %d\n", 
          __FUNCTION__, home.tran.x, home.tran.y, home.tran.z, home.a, home.b, home.c, 
          home.u, home.v, home.w, retval);
    }
    return retval;
}

int lbvTrajSetScale(double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    lbvmotCommand.command = LBVMOT_FEED_SCALE;
    lbvmotCommand.scale = scale;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetRapidScale(double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    lbvmotCommand.command = LBVMOT_RAPID_SCALE;
    lbvmotCommand.scale = scale;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetSpindleScale(int spindle, double scale)
{
    if (scale < 0.0) {
	scale = 0.0;
    }

    lbvmotCommand.command = LBVMOT_SPINDLE_SCALE;
    lbvmotCommand.scale = scale;
    lbvmotCommand.spindle = spindle;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetFOEnable(unsigned char mode)
{
    lbvmotCommand.command = LBVMOT_FS_ENABLE;
    lbvmotCommand.mode = mode;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetFHEnable(unsigned char mode)
{
    lbvmotCommand.command = LBVMOT_FH_ENABLE;
    lbvmotCommand.mode = mode;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetSOEnable(unsigned char mode)
{
    lbvmotCommand.command = LBVMOT_SS_ENABLE;
    lbvmotCommand.mode = mode;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetAFEnable(unsigned char enable)
{
    lbvmotCommand.command = LBVMOT_AF_ENABLE;

    if ( enable ) {
	lbvmotCommand.flags = 1;
    } else {
	lbvmotCommand.flags = 0;
    }
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetMotionId(int id)
{

    if (LBV_DEBUG_MOTION_TIME & lbv_debug) {
	if (id != TrajConfig.MotionId) {
	    rcs_print("Outgoing motion id is %d.\n", id);
	}
    }

    TrajConfig.MotionId = id;

    return 0;
}

int lbvTrajInit()
{
    int retval = 0;

    TrajConfig.Inited = 0;
    TrajConfig.Joints = 0;
    TrajConfig.MaxAccel = DBL_MAX;
    TrajConfig.DeprecatedAxes = 0;
    TrajConfig.AxisMask = 0;
    TrajConfig.LinearUnits = 1.0;
    TrajConfig.AngularUnits = 1.0;
    TrajConfig.MotionId = 0;
    TrajConfig.MaxVel = DEFAULT_TRAJ_MAX_VELOCITY;

    // init lbvmot interface
    if (!JointOrTrajInited()) {
	usrmotIniLoad(lbv_inifile);
	if (0 != usrmotInit("lbv2_task")) {
	    return -1;
	}
    }
    TrajConfig.Inited = 1;
    // initialize parameters from ini file
    if (0 != iniTraj(lbv_inifile)) {
	retval = -1;
    }
    return retval;
}

int lbvTrajHalt()
{
    TrajConfig.Inited = 0;

    if (!JointOrTrajInited()) {
	usrmotExit();		// ours is final exit
    }

    return 0;
}

int lbvTrajEnable()
{
    lbvmotCommand.command = LBVMOT_ENABLE;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajDisable()
{
    lbvmotCommand.command = LBVMOT_DISABLE;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajAbort()
{
    lbvmotCommand.command = LBVMOT_ABORT;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajPause()
{
    lbvmotCommand.command = LBVMOT_PAUSE;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajReverse()
{
    lbvmotCommand.command = LBVMOT_REVERSE;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajForward()
{
    lbvmotCommand.command = LBVMOT_FORWARD;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajStep()
{
    lbvmotCommand.command = LBVMOT_STEP;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajResume()
{
    lbvmotCommand.command = LBVMOT_RESUME;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajDelay(double delay)
{
    /* nothing need be done here - it's done in task controller */

    return 0;
}

double lbvTrajGetLinearUnits()
{
    return TrajConfig.LinearUnits;
}

double lbvTrajGetAngularUnits()
{
    return TrajConfig.AngularUnits;
}

int lbvTrajSetOffset(LbvPose tool_offset)
{
    lbvmotCommand.command = LBVMOT_SET_OFFSET;
    lbvmotCommand.tool_offset = tool_offset;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetSpindleSync(int spindle, double fpr, bool wait_for_index)
{
    lbvmotCommand.command = LBVMOT_SET_SPINDLESYNC;
    lbvmotCommand.spindle = spindle;
    lbvmotCommand.spindlesync = fpr;
    lbvmotCommand.flags = wait_for_index;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajSetTermCond(int cond, double tolerance)
{
    lbvmotCommand.command = LBVMOT_SET_TERM_COND;
    // Direct passthrough since TP can handle the distinction now
    lbvmotCommand.termCond = cond;
    lbvmotCommand.tolerance = tolerance;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajLinearMove(LbvPose end, int type, double vel, double ini_maxvel, double acc,
                      int indexer_jnum)
{
#ifdef ISNAN_TRAP
    if (std::isnan(end.tran.x) || std::isnan(end.tran.y) || std::isnan(end.tran.z) ||
        std::isnan(end.a) || std::isnan(end.b) || std::isnan(end.c) ||
        std::isnan(end.u) || std::isnan(end.v) || std::isnan(end.w)) {
	printf("std::isnan error in lbvTrajLinearMove()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    lbvmotCommand.command = LBVMOT_SET_LINE;

    lbvmotCommand.pos = end;

    lbvmotCommand.id = TrajConfig.MotionId;
    lbvmotCommand.motion_type = type;
    lbvmotCommand.vel = vel;
    lbvmotCommand.ini_maxvel = ini_maxvel;
    lbvmotCommand.acc = acc;
    lbvmotCommand.turn = indexer_jnum;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajCircularMove(LbvPose end, PM_CARTESIAN center,
			PM_CARTESIAN normal, int turn, int type, double vel, double ini_maxvel, double acc)
{
#ifdef ISNAN_TRAP
    if (std::isnan(end.tran.x) || std::isnan(end.tran.y) || std::isnan(end.tran.z) ||
	std::isnan(end.a) || std::isnan(end.b) || std::isnan(end.c) ||
	std::isnan(end.u) || std::isnan(end.v) || std::isnan(end.w) ||
	std::isnan(center.x) || std::isnan(center.y) || std::isnan(center.z) ||
	std::isnan(normal.x) || std::isnan(normal.y) || std::isnan(normal.z)) {
	printf("std::isnan error in lbvTrajCircularMove()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    lbvmotCommand.command = LBVMOT_SET_CIRCLE;

    lbvmotCommand.pos = end;
    lbvmotCommand.motion_type = type;

    lbvmotCommand.center.x = center.x;
    lbvmotCommand.center.y = center.y;
    lbvmotCommand.center.z = center.z;

    lbvmotCommand.normal.x = normal.x;
    lbvmotCommand.normal.y = normal.y;
    lbvmotCommand.normal.z = normal.z;

    lbvmotCommand.turn = turn;
    lbvmotCommand.id = TrajConfig.MotionId;

    lbvmotCommand.vel = vel;
    lbvmotCommand.ini_maxvel = ini_maxvel;
    lbvmotCommand.acc = acc;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajClearProbeTrippedFlag()
{
    lbvmotCommand.command = LBVMOT_CLEAR_PROBE_FLAGS;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajProbe(LbvPose pos, int type, double vel, double ini_maxvel, double acc, unsigned char probe_type)
{
#ifdef ISNAN_TRAP
    if (std::isnan(pos.tran.x) || std::isnan(pos.tran.y) || std::isnan(pos.tran.z) ||
        std::isnan(pos.a) || std::isnan(pos.b) || std::isnan(pos.c) ||
        std::isnan(pos.u) || std::isnan(pos.v) || std::isnan(pos.w)) {
	printf("std::isnan error in lbvTrajProbe()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    lbvmotCommand.command = LBVMOT_PROBE;
    lbvmotCommand.pos = pos;
    lbvmotCommand.id = TrajConfig.MotionId;
    lbvmotCommand.motion_type = type;
    lbvmotCommand.vel = vel;
    lbvmotCommand.ini_maxvel = ini_maxvel;
    lbvmotCommand.acc = acc;
    lbvmotCommand.probe_type = probe_type;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvTrajRigidTap(LbvPose pos, double vel, double ini_maxvel, double acc, double scale)
{
#ifdef ISNAN_TRAP
    if (std::isnan(pos.tran.x) || std::isnan(pos.tran.y) || std::isnan(pos.tran.z)) {
	printf("std::isnan error in lbvTrajRigidTap()\n");
	return 0;		// ignore it for now, just don't send it
    }
#endif

    lbvmotCommand.command = LBVMOT_RIGID_TAP;
    lbvmotCommand.pos.tran = pos.tran;
    lbvmotCommand.id = TrajConfig.MotionId;
    lbvmotCommand.vel = vel;
    lbvmotCommand.ini_maxvel = ini_maxvel;
    lbvmotCommand.acc = acc;
    lbvmotCommand.scale = scale;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}


static int last_id = 0;
static int last_id_printed = 0;
static int last_status = 0;
static double last_id_time;

int lbvTrajUpdate(LBV_TRAJ_STAT * stat)
{
    int joint, enables;

    stat->joints = TrajConfig.Joints;
    stat->spindles = TrajConfig.Spindles;
    stat->deprecated_axes = TrajConfig.DeprecatedAxes;
    stat->axis_mask = TrajConfig.AxisMask;
    stat->linearUnits = TrajConfig.LinearUnits;
    stat->angularUnits = TrajConfig.AngularUnits;

    stat->mode =
	lbvmotStatus.
	motionFlag & LBVMOT_MOTION_TELEOP_BIT ? LBV_TRAJ_MODE_TELEOP
	: (lbvmotStatus.
	   motionFlag & LBVMOT_MOTION_COORD_BIT ? LBV_TRAJ_MODE_COORD :
	   LBV_TRAJ_MODE_FREE);

    /* enabled if motion enabled and all joints enabled */
    stat->enabled = 0;		/* start at disabled */
    if (lbvmotStatus.motionFlag & LBVMOT_MOTION_ENABLE_BIT) {
	for (joint = 0; joint < TrajConfig.Joints; joint++) {
/*! \todo Another #if 0 */
#if 0				/*! \todo FIXME - the axis flag has been moved to the joint struct */
	    if (!lbvmotStatus.axisFlag[axis] & LBVMOT_JOINT_ENABLE_BIT) {
		break;
	    }
#endif
	    /* got here, then all are enabled */
	    stat->enabled = 1;
	}
    }

    stat->inpos = lbvmotStatus.motionFlag & LBVMOT_MOTION_INPOS_BIT;
    stat->queue = lbvmotStatus.depth;
    stat->activeQueue = lbvmotStatus.activeDepth;
    stat->queueFull = lbvmotStatus.queueFull;
    stat->id = lbvmotStatus.id;
    stat->motion_type = lbvmotStatus.motionType;
    stat->distance_to_go = lbvmotStatus.distance_to_go;
    stat->dtg = lbvmotStatus.dtg;
    stat->current_vel = lbvmotStatus.current_vel;
    if (LBV_DEBUG_MOTION_TIME & lbv_debug) {
	if (stat->id != last_id) {
	    if (last_id != last_id_printed) {
		rcs_print("Motion id %d took %f seconds.\n", last_id,
			  etime() - last_id_time);
		last_id_printed = last_id;
	    }
	    last_id = stat->id;
	    last_id_time = etime();
	}
    }

    stat->paused = lbvmotStatus.paused;
    stat->scale = lbvmotStatus.feed_scale;
    stat->rapid_scale = lbvmotStatus.rapid_scale;

    stat->position = lbvmotStatus.carte_pos_cmd;

    stat->actualPosition = lbvmotStatus.carte_pos_fb;

    stat->velocity = lbvmotStatus.vel;
    stat->acceleration = lbvmotStatus.acc;
    stat->maxAcceleration = TrajConfig.MaxAccel;

    if (lbvmotStatus.motionFlag & LBVMOT_MOTION_ERROR_BIT) {
	stat->status = RCS_ERROR;
    } else if (stat->inpos && (stat->queue == 0)) {
	stat->status = RCS_DONE;
    } else {
	stat->status = RCS_EXEC;
    }

    if (LBV_DEBUG_MOTION_TIME & lbv_debug) {
	if (stat->status == RCS_DONE && last_status != RCS_DONE
	    && stat->id != last_id_printed) {
	    rcs_print("Motion id %d took %f seconds.\n", last_id,
		      etime() - last_id_time);
	    last_id_printed = last_id = stat->id;
	    last_id_time = etime();
	}
    }

    stat->probedPosition = lbvmotStatus.probedPos;

    stat->probeval = lbvmotStatus.probeVal;
    stat->probing = lbvmotStatus.probing;
    stat->probe_tripped = lbvmotStatus.probeTripped;
    
    if (lbvmotStatus.motionFlag & LBVMOT_MOTION_COORD_BIT)
        enables = lbvmotStatus.enables_queued;
    else
        enables = lbvmotStatus.enables_new;
    
    stat->feed_override_enabled = enables & FS_ENABLED;
    stat->adaptive_feed_enabled = enables & AF_ENABLED;
    stat->feed_hold_enabled = enables & FH_ENABLED;

    if (new_config) {
	stat->cycleTime = lbvmotConfig.trajCycleTime;
	stat->kinematics_type = lbvmotConfig.kinType;
	stat->maxVelocity = lbvmotConfig.limitVel;
    }

    return 0;
}


int setup_inihal(void) {
    // Must be called after lbvTrajInit(), which loads the number of
    // joints from the ini file.
    if (lbvmotion_initialized != 1) {
        rcs_print_error("%s: lbvMotionInit() has not completed, can't setup inihal\n", __FUNCTION__);
        return -1;
    }

    if (ini_hal_init(TrajConfig.Joints)) {
        rcs_print_error("%s: ini_hal_init(%d) failed\n", __FUNCTION__, TrajConfig.Joints);
        return -1;
    }

    if (ini_hal_init_pins(TrajConfig.Joints)) {
        rcs_print_error("%s: ini_hal_init_pins(%d) failed\n", __FUNCTION__, TrajConfig.Joints);
        return -1;
    }

    return 0;
}


int lbvPositionLoad() {
    double positions[LBVMOT_MAX_JOINTS];
    IniFile ini;
    ini.Open(lbv_inifile);
    const char *posfile = ini.Find("POSITION_FILE", "TRAJ");
    ini.Close();
    if(!posfile || !posfile[0]) return 0;
    FILE *f = fopen(posfile, "r");
    if(!f) return 0;
    for(int i=0; i<LBVMOT_MAX_JOINTS; i++) {
	int r = fscanf(f, "%lf", &positions[i]);
	if(r != 1) {
            fclose(f);
            rcs_print("%s: failed to load joint %d position from %s, ignoring\n", __FUNCTION__, i, posfile);
            return -1;
        }
    }
    fclose(f);
    int result = 0;
    for(int i=0; i<LBVMOT_MAX_JOINTS; i++) {
	if(lbvJointSetMotorOffset(i, -positions[i]) != 0) {
            rcs_print("%s: failed to set joint %d position (%.6f) from %s, ignoring\n", __FUNCTION__, i, positions[i], posfile);
            result = -1;
        }
    }
    return result;
}


int lbvPositionSave() {
    IniFile ini;
    const char *posfile;

    ini.Open(lbv_inifile);
    try {
        posfile = ini.Find("POSITION_FILE", "TRAJ");
    } catch (IniFile::Exception e) {
        ini.Close();
        return -1;
    }
    ini.Close();

    if(!posfile || !posfile[0]) return 0;
    // like the var file, make sure the posfile is recreated according to umask
    unlink(posfile);
    FILE *f = fopen(posfile, "w");
    if(!f) return -1;
    for(int i=0; i<LBVMOT_MAX_JOINTS; i++) {
	int r = fprintf(f, "%.17f\n", lbvmotStatus.joint_status[i].pos_fb);
	if(r < 0) { fclose(f); return -1; }
    }
    fclose(f);
    return 0;
}

// LBV_MOTION functions

// This function gets called by Task from lbvtask_startup().
// lbvtask_startup() calls this function in a loop, retrying it until
// it succeeds or until the retries time out.
int lbvMotionInit()
{
    int r;
    int joint, axis;
    
    r = lbvTrajInit(); // we want to check Traj first, the sane defaults for units are there
    // it also determines the number of existing joints, and axes
    if (r != 0) {
        rcs_print("%s: lbvTrajInit failed\n", __FUNCTION__);
        return -1;
    }

    for (joint = 0; joint < TrajConfig.Joints; joint++) {
	if (0 != lbvJointInit(joint)) {
            rcs_print("%s: lbvJointInit(%d) failed\n", __FUNCTION__, joint);
            return -1;
	}
    }

    for (axis = 0; axis < LBVMOT_MAX_AXIS; axis++) {
        if (TrajConfig.AxisMask & (1<<axis)) {
	    if (0 != lbvAxisInit(axis)) {
                rcs_print("%s: lbvAxisInit(%d) failed\n", __FUNCTION__, axis);
                return -1;
	    }
	}
    }

    // Ignore errors from lbvPositionLoad(), because what are you going to do?
    (void)lbvPositionLoad();

    lbvmotion_initialized = 1;

    return 0;
}

int lbvMotionHalt()
{
    int r1, r2, r3, r4, r5;
    int t;

    r1 = -1;
    for (t = 0; t < LBVMOT_MAX_JOINTS; t++) {
	if (0 == lbvJointHalt(t)) {
	    r1 = 0;		// at least one is okay
	}
    }

    r2 = lbvTrajDisable();
    r3 = lbvTrajHalt();
    r4 = lbvPositionSave();
    r5 = ini_hal_exit();
    lbvmotion_initialized = 0;

    return (r1 == 0 && r2 == 0 && r3 == 0 && r4 == 0 && r5 == 0) ? 0 : -1;
}

int lbvMotionAbort()
{
    int r1;
    int r2;
    int r3 = 0;
    int t;

    r1 = -1;
    for (t = 0; t < LBVMOT_MAX_JOINTS; t++) {
	if (0 == lbvJointAbort(t)) {
	    r1 = 0;		// at least one is okay
	}
    }

    r2 = lbvTrajAbort();

    return (r1 == 0 && r2 == 0 && r3 == 0) ? 0 : -1;
}

int lbvMotionSetDebug(int debug)
{
    lbvmotCommand.command = LBVMOT_SET_DEBUG;
    lbvmotCommand.debug = debug;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

/*! \function lbvMotionSetAout()
    
    This function sends a LBVMOT_SET_AOUT message to the motion controller.
    That one plans a AOUT command when motion starts or right now.

    @parameter	index	which output gets modified
    @parameter	now	wheather change is imediate or synched with motion
    @parameter	start	value set at start of motion
    @parameter	end	value set at end of motion
*/
int lbvMotionSetAout(unsigned char index, double start, double end, unsigned char now)
{
    lbvmotCommand.command = LBVMOT_SET_AOUT;
    lbvmotCommand.now = now;
    lbvmotCommand.out = index;
  /*! \todo FIXME-- if this works, set up some dedicated cmd fields instead of
     borrowing these */
    lbvmotCommand.minLimit = start;
    lbvmotCommand.maxLimit = end;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

/*! \function lbvMotionSetDout()
    
    This function sends a LBVMOT_SET_DOUT message to the motion controller.
    That one plans a DOUT command when motion starts or right now.

    @parameter	index	which output gets modified
    @parameter	now	wheather change is imediate or synched with motion
    @parameter	start	value set at start of motion
    @parameter	end	value set at end of motion
*/
int lbvMotionSetDout(unsigned char index, unsigned char start,
		     unsigned char end, unsigned char now)
{
    lbvmotCommand.command = LBVMOT_SET_DOUT;
    lbvmotCommand.now = now;
    lbvmotCommand.out = index;
    lbvmotCommand.start = start;
    lbvmotCommand.end = end;

    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleAbort(int spindle)
{
    return lbvSpindleOff(spindle);
}

int lbvSpindleSpeed(int spindle, double speed, double css_factor, double offset)
{
    if (lbvmotStatus.spindle_status[spindle].speed == 0){
        return 0;} //spindle stopped, not updating speed */

    return lbvSpindleOn(spindle, speed, css_factor, offset);
}

int lbvSpindleOrient(int spindle, double orientation, int mode)
{
    lbvmotCommand.command = LBVMOT_SPINDLE_ORIENT;
    lbvmotCommand.spindle = spindle;
    lbvmotCommand.orientation = orientation;
    lbvmotCommand.mode = mode;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}


int lbvSpindleOn(int spindle, double speed, double css_factor, double offset, int wait_for_at_speed)
{

    lbvmotCommand.command = LBVMOT_SPINDLE_ON;
    lbvmotCommand.spindle = spindle;
    lbvmotCommand.vel = speed;
    lbvmotCommand.ini_maxvel = css_factor;
    lbvmotCommand.acc = offset;
    lbvmotCommand.wait_for_spindle_at_speed = wait_for_at_speed;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleOff(int spindle)
{
    lbvmotCommand.command = LBVMOT_SPINDLE_OFF;
    lbvmotCommand.spindle = spindle;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleBrakeRelease(int spindle)
{
    lbvmotCommand.command = LBVMOT_SPINDLE_BRAKE_RELEASE;
    lbvmotCommand.spindle = spindle;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleBrakeEngage(int spindle)
{
    lbvmotCommand.command = LBVMOT_SPINDLE_BRAKE_ENGAGE;
    lbvmotCommand.spindle = spindle;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleIncrease(int spindle)
{
    lbvmotCommand.command = LBVMOT_SPINDLE_INCREASE;
    lbvmotCommand.spindle = spindle;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleDecrease(int spindle)
{
    lbvmotCommand.command = LBVMOT_SPINDLE_DECREASE;
    lbvmotCommand.spindle = spindle;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSpindleConstant(int spindle)
{
    return 0; // nothing to do
}

int lbvSpindleUpdate(LBV_SPINDLE_STAT stat[], int num_spindles){
	int s;
	int enables;
    if (lbvmotStatus.motionFlag & LBVMOT_MOTION_COORD_BIT)
        enables = lbvmotStatus.enables_queued;
    else
        enables = lbvmotStatus.enables_new;

    for (s = 0; s < num_spindles; s++){
		stat[s].spindle_override_enabled = enables & SS_ENABLED;
		stat[s].enabled = lbvmotStatus.spindle_status[s].speed != 0;
		stat[s].speed = lbvmotStatus.spindle_status[s].speed;
		stat[s].brake = lbvmotStatus.spindle_status[s].brake;
		stat[s].direction = lbvmotStatus.spindle_status[s].direction;
		stat[s].orient_state = lbvmotStatus.spindle_status[s].orient_state;
		stat[s].orient_fault = lbvmotStatus.spindle_status[s].orient_fault;
		stat[s].spindle_scale = lbvmotStatus.spindle_status[s].scale;
    }
    return 0;
}

int lbvMotionUpdate(LBV_MOTION_STAT * stat)
{
    int r1, r2, r3, r4;
    int joint;
    int error;
    int exec;
    int dio, aio;

    // read the lbvmot status
    if (0 != usrmotReadLbvmotStatus(&lbvmotStatus)) {
	return -1;
    }
    new_config = 0;
    if (lbvmotStatus.config_num != lbvmotConfig.config_num) {
	if (0 != usrmotReadLbvmotConfig(&lbvmotConfig)) {
	    return -1;
	}
	new_config = 1;
    }

    if (get_lbvmot_debug_info) {
	if (0 != usrmotReadLbvmotDebug(&lbvmotDebug)) {
	    return -1;
	}
    }
    // read the lbvmot error
    if (0 != usrmotReadLbvmotError(errorString)) {
	// no error, so ignore
    } else {
	// an error to report
	lbvOperatorError(0, "%s", errorString);
    }

    // save the heartbeat and command number locally,
    // for use with lbvMotionUpdate
    localMotionHeartbeat = lbvmotStatus.heartbeat;
    localMotionCommandType = lbvmotStatus.commandEcho;	/*! \todo FIXME-- not NML one! */
    localMotionEchoSerialNumber = lbvmotStatus.commandNumEcho;

    r3 = lbvTrajUpdate(&stat->traj);
    r1 = lbvJointUpdate(&stat->joint[0], stat->traj.joints);
    r2 = lbvAxisUpdate(&stat->axis[0], stat->traj.axis_mask);
    r3 = lbvTrajUpdate(&stat->traj);
    r4 = lbvSpindleUpdate(&stat->spindle[0], stat->traj.spindles);
    stat->heartbeat = localMotionHeartbeat;
    stat->command_type = localMotionCommandType;
    stat->echo_serial_number = localMotionEchoSerialNumber;
    stat->debug = lbvmotConfig.debug;

    for (dio = 0; dio < LBVMOT_MAX_DIO; dio++) {
	stat->synch_di[dio] = lbvmotStatus.synch_di[dio];
	stat->synch_do[dio] = lbvmotStatus.synch_do[dio];
    }

    for (aio = 0; aio < LBVMOT_MAX_AIO; aio++) {
	stat->analog_input[aio] = lbvmotStatus.analog_input[aio];
	stat->analog_output[aio] = lbvmotStatus.analog_output[aio];
    }

    stat->numExtraJoints=lbvmotStatus.numExtraJoints;

    // set the status flag
    error = 0;
    exec = 0;

    // FIXME-AJ: joints not axes
    for (joint = 0; joint < stat->traj.joints; joint++) {
	if (stat->joint[joint].status == RCS_ERROR) {
	    error = 1;
	    break;
	}
	if (stat->joint[joint].status == RCS_EXEC) {
	    exec = 1;
	    break;
	}
    }
    if (stat->traj.status == RCS_ERROR) {
	error = 1;
    } else if (stat->traj.status == RCS_EXEC) {
	exec = 1;
    }

    if (error) {
	stat->status = RCS_ERROR;
    } else if (exec) {
	stat->status = RCS_EXEC;
    } else {
	stat->status = RCS_DONE;
    }
    return (r1 == 0 && r2 == 0 && r3 == 0 && r4 == 0) ? 0 : -1;
}

int lbvSetupArcBlends(int arcBlendEnable,
        int arcBlendFallbackEnable,
        int arcBlendOptDepth,
        int arcBlendGapCycles,
        double arcBlendRampFreq,
        double arcBlendTangentKinkRatio) {

    lbvmotCommand.command = LBVMOT_SETUP_ARC_BLENDS;
    lbvmotCommand.arcBlendEnable = arcBlendEnable;
    lbvmotCommand.arcBlendFallbackEnable = arcBlendFallbackEnable;
    lbvmotCommand.arcBlendOptDepth = arcBlendOptDepth;
    lbvmotCommand.arcBlendGapCycles = arcBlendGapCycles;
    lbvmotCommand.arcBlendRampFreq = arcBlendRampFreq;
    lbvmotCommand.arcBlendTangentKinkRatio = arcBlendTangentKinkRatio;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSetMaxFeedOverride(double maxFeedScale) {
    lbvmotCommand.command = LBVMOT_SET_MAX_FEED_OVERRIDE;
    lbvmotCommand.maxFeedScale = maxFeedScale;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvSetProbeErrorInhibit(int j_inhibit, int h_inhibit) {
    lbvmotCommand.command = LBVMOT_SET_PROBE_ERR_INHIBIT;
    lbvmotCommand.probe_jog_err_inhibit = j_inhibit;
    lbvmotCommand.probe_home_err_inhibit = h_inhibit;
    return usrmotWriteLbvmotCommand(&lbvmotCommand);
}

int lbvGetExternalOffsetApplied(void) {
    return lbvmotStatus.external_offsets_applied;
}

LbvPose lbvGetExternalOffsets(void) {
    return lbvmotStatus.eoffset_pose;
}
