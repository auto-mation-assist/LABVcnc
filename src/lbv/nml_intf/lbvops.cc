/********************************************************************
* Description: lbvops.cc
*   Initialization and other ad hoc functions for NML. This complements
*   the auto-generated lbv.cc, which contains all the rote update
*   methods for the message classes.
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

#include "lbv.hh"
#include "lbv_nml.hh"

LBV_AXIS_STAT::LBV_AXIS_STAT():
LBV_AXIS_STAT_MSG(LBV_AXIS_STAT_TYPE, sizeof(LBV_AXIS_STAT))
{
}

LBV_JOINT_STAT::LBV_JOINT_STAT():
LBV_JOINT_STAT_MSG(LBV_JOINT_STAT_TYPE, sizeof(LBV_JOINT_STAT))
{
    jointType = LBV_LINEAR;
    units = 1.0;
    backlash = 0.0;
    minPositionLimit = -1.0;
    maxPositionLimit = 1.0;
    minFerror = 1.0;
    maxFerror = 1.0;
    ferrorCurrent = 0.0;
    ferrorHighMark = 0.0;
    output = 0.0;
    input = 0.0;
    velocity = 0.0;
    inpos = 1;
    homing = 0;
    homed = 0;
    enabled = 0;
    fault = 0;
    minSoftLimit = 0;
    maxSoftLimit = 0;
    minHardLimit = 0;
    maxHardLimit = 0;
    overrideLimits = 0;
}

LBV_TRAJ_STAT::LBV_TRAJ_STAT():
LBV_TRAJ_STAT_MSG(LBV_TRAJ_STAT_TYPE, sizeof(LBV_TRAJ_STAT))
{
    linearUnits = 1.0;
    angularUnits = 1.0;
    cycleTime = 0.0;
    joints = 1;
    deprecated_axes = 1;
    axis_mask = 1;
    mode = LBV_TRAJ_MODE_FREE;
    enabled = OFF;
    inpos = ON;
    queue = 0;
    activeQueue = 0;
    queueFull = OFF;
    id = 0;
    paused = OFF;
    scale = 0.0;
    rapid_scale = 0.0;

    ZERO_LBV_POSE(position);
    ZERO_LBV_POSE(actualPosition);

    velocity = 1.0;
    acceleration = 1.0;
    maxVelocity = 1.0;
    maxAcceleration = 1.0;

    ZERO_LBV_POSE(probedPosition);
    probe_tripped = OFF;
    probing = OFF;
    probeval = 0;
    
    ZERO_LBV_POSE(dtg);
    distance_to_go = 0.0;
    kinematics_type = 0;
    motion_type = 0;
    current_vel = 0.0;
    feed_override_enabled = OFF;
    adaptive_feed_enabled = OFF;
    feed_hold_enabled = OFF;
}

LBV_MOTION_STAT::LBV_MOTION_STAT():
LBV_MOTION_STAT_MSG(LBV_MOTION_STAT_TYPE, sizeof(LBV_MOTION_STAT))
{
    int i;

    for (i = 0; i < LBVMOT_MAX_DIO; i++) {
	synch_di[i] = 0;
	synch_do[i] = 0;
    }

    for (i = 0; i < LBVMOT_MAX_AIO; i++) {
	analog_input[i] = 0.0;
	analog_output[i] = 0.0;
    }

    debug = 0;
};

LBV_TASK_STAT::LBV_TASK_STAT():
LBV_TASK_STAT_MSG(LBV_TASK_STAT_TYPE, sizeof(LBV_TASK_STAT))
{
    int t;

    mode = LBV_TASK_MODE_MANUAL;
    state = LBV_TASK_STATE_ESTOP;
    execState = LBV_TASK_EXEC_DONE;
    interpState = LBV_TASK_INTERP_IDLE;
    callLevel = 0;
    motionLine = 0;
    currentLine = 0;
    readLine = 0;
    optional_stop_state = OFF;
    block_delete_state = OFF;
    input_timeout = OFF;
    file[0] = 0;
    command[0] = 0;

    ZERO_LBV_POSE(g5x_offset);
    g5x_index = 0;
    ZERO_LBV_POSE(g92_offset);
    ZERO_LBV_POSE(toolOffset);

    rotation_xy = 0.0;

    for (t = 0; t < ACTIVE_G_CODES; t++)
	activeGCodes[t] = -1;
    for (t = 0; t < ACTIVE_M_CODES; t++)
	activeMCodes[t] = -1;
    for (t = 0; t < ACTIVE_SETTINGS; t++)
	activeSettings[t] = 0.0;

    programUnits = CANON_UNITS_MM;
    interpreter_errcode = 0;
    task_paused = 0;
    delayLeft = 0.0;
    queuedMDIcommands = 0;
}

LBV_TOOL_STAT::LBV_TOOL_STAT():
LBV_TOOL_STAT_MSG(LBV_TOOL_STAT_TYPE, sizeof(LBV_TOOL_STAT))
{
    int t;

    pocketPrepped = 0;
    toolInSpindle = 0;

    for (t = 0; t < CANON_POCKETS_MAX; t++) {
	toolTable[t].toolno = 0;
    toolTable[t].pocketno = 0;
        ZERO_LBV_POSE(toolTable[t].offset);
	toolTable[t].diameter = 0.0;
	toolTable[t].orientation = 0;
	toolTable[t].frontangle = 0.0;
	toolTable[t].backangle = 0.0;
    }
}

LBV_AUX_STAT::LBV_AUX_STAT():
LBV_AUX_STAT_MSG(LBV_AUX_STAT_TYPE, sizeof(LBV_AUX_STAT))
{
    estop = 1;
}

LBV_SPINDLE_STAT::LBV_SPINDLE_STAT():
LBV_SPINDLE_STAT_MSG(LBV_SPINDLE_STAT_TYPE, sizeof(LBV_SPINDLE_STAT))
{
    speed = 0.0;
    direction = 0;
    brake = 1;
    increasing = 0;
    enabled = 0;
    spindle_scale = 1.0;
    spindle_override_enabled = 0;

}

LBV_COOLANT_STAT::LBV_COOLANT_STAT():LBV_COOLANT_STAT_MSG(LBV_COOLANT_STAT_TYPE,
		     sizeof
		     (LBV_COOLANT_STAT))
{
    mist = 0;
    flood = 0;
}

LBV_LUBE_STAT::LBV_LUBE_STAT():
LBV_LUBE_STAT_MSG(LBV_LUBE_STAT_TYPE, sizeof(LBV_LUBE_STAT))
{
    on = 0;
    level = 1;
}

// overload = , since class has array elements
LBV_TOOL_STAT LBV_TOOL_STAT::operator =(LBV_TOOL_STAT s)
{
    int t;

    pocketPrepped = s.pocketPrepped;
    toolInSpindle = s.toolInSpindle;

    for (t = 0; t < CANON_POCKETS_MAX; t++) {
	toolTable[t].toolno = s.toolTable[t].toolno;
    toolTable[t].pocketno = s.toolTable[t].pocketno;
	toolTable[t].offset = s.toolTable[t].offset;
	toolTable[t].diameter = s.toolTable[t].diameter;
	toolTable[t].frontangle = s.toolTable[t].frontangle;
	toolTable[t].backangle = s.toolTable[t].backangle;
	toolTable[t].orientation = s.toolTable[t].orientation;
    }

    return s;
}

LBV_STAT::LBV_STAT():LBV_STAT_MSG(LBV_STAT_TYPE, sizeof(LBV_STAT))
{
}
