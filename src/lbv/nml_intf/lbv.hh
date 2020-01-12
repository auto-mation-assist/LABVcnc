/********************************************************************
* Description: lbv.hh
*   Declarations for LBV NML vocabulary
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
#ifndef LBV_HH
#define LBV_HH

#include "config.h"
#include "lbvmotcfg.h"		// LBV_JOINT_MAX, LBV_AXIS_MAX
#include "nml_type.hh"
#include "motion_types.h"
#include <stdint.h>

// Forward class declarations
class LBV_JOINT_STAT;
class LBV_AXIS_STAT;
class LBV_TRAJ_STAT;
class LBV_MOTION_STAT;
class LBV_TASK_STAT;
class LBV_TOOL_STAT;
class LBV_AUX_STAT;
class LBV_SPINDLE_STAT;
class LBV_COOLANT_STAT;
class LBV_LUBE_STAT;
class LBV_IO_STAT;
class LBV_STAT;
class CMS;
class RCS_CMD_CHANNEL;
class RCS_STAT_CHANNEL;
class NML;
struct LbvPose;
struct PM_CARTESIAN;

// ---------------------
// LBV TYPE DECLARATIONS
// ---------------------

// NML for base LBV

#define LBV_OPERATOR_ERROR_TYPE                      ((NMLTYPE) 11)
#define LBV_OPERATOR_TEXT_TYPE                       ((NMLTYPE) 12)
#define LBV_OPERATOR_DISPLAY_TYPE                    ((NMLTYPE) 13)

#define LBV_NULL_TYPE                                ((NMLTYPE) 21)

#define LBV_SET_DEBUG_TYPE                           ((NMLTYPE) 22)

#define LBV_SYSTEM_CMD_TYPE                          ((NMLTYPE) 30)

// NML for LBV_JOINT

#define LBV_JOINT_SET_JOINT_TYPE                       ((NMLTYPE) 101)
#define LBV_JOINT_SET_UNITS_TYPE                      ((NMLTYPE) 102)
#define LBV_JOINT_SET_MIN_POSITION_LIMIT_TYPE         ((NMLTYPE) 107)
#define LBV_JOINT_SET_MAX_POSITION_LIMIT_TYPE         ((NMLTYPE) 108)
#define LBV_JOINT_SET_FERROR_TYPE                     ((NMLTYPE) 111)
#define LBV_JOINT_SET_HOMING_PARAMS_TYPE              ((NMLTYPE) 112)
#define LBV_JOINT_SET_MIN_FERROR_TYPE                 ((NMLTYPE) 115)
#define LBV_JOINT_SET_MAX_VELOCITY_TYPE               ((NMLTYPE) 116)
#define LBV_JOINT_INIT_TYPE                           ((NMLTYPE) 118)
#define LBV_JOINT_HALT_TYPE                           ((NMLTYPE) 119)
#define LBV_JOINT_ABORT_TYPE                          ((NMLTYPE) 120)
#define LBV_JOINT_ENABLE_TYPE                         ((NMLTYPE) 121)
#define LBV_JOINT_DISABLE_TYPE                        ((NMLTYPE) 122)
#define LBV_JOINT_HOME_TYPE                           ((NMLTYPE) 123)
#define LBV_JOG_CONT_TYPE                             ((NMLTYPE) 124)
#define LBV_JOG_INCR_TYPE                             ((NMLTYPE) 125)
#define LBV_JOG_ABS_TYPE                              ((NMLTYPE) 126)
#define LBV_JOINT_ACTIVATE_TYPE                       ((NMLTYPE) 127)
#define LBV_JOINT_DEACTIVATE_TYPE                     ((NMLTYPE) 128)
#define LBV_JOINT_OVERRIDE_LIMITS_TYPE                ((NMLTYPE) 129)
#define LBV_JOINT_LOAD_COMP_TYPE                      ((NMLTYPE) 131)
#define LBV_JOINT_SET_BACKLASH_TYPE                   ((NMLTYPE) 134)
#define LBV_JOINT_UNHOME_TYPE                         ((NMLTYPE) 135)
#define LBV_JOG_STOP_TYPE                             ((NMLTYPE) 136)

#define LBV_JOINT_STAT_TYPE                          ((NMLTYPE) 198)
#define LBV_AXIS_STAT_TYPE                           ((NMLTYPE) 199)

// NML for LBV_TRAJ

// defs for termination conditions
#define LBV_TRAJ_TERM_COND_STOP  0
#define LBV_TRAJ_TERM_COND_EXACT 1
#define LBV_TRAJ_TERM_COND_BLEND 2

#define LBV_TRAJ_SET_AXES_TYPE                       ((NMLTYPE) 201)
#define LBV_TRAJ_SET_UNITS_TYPE                      ((NMLTYPE) 202)
#define LBV_TRAJ_SET_CYCLE_TIME_TYPE                 ((NMLTYPE) 203)
#define LBV_TRAJ_SET_MODE_TYPE                       ((NMLTYPE) 204)
#define LBV_TRAJ_SET_VELOCITY_TYPE                   ((NMLTYPE) 205)
#define LBV_TRAJ_SET_ACCELERATION_TYPE               ((NMLTYPE) 206)
#define LBV_TRAJ_SET_MAX_VELOCITY_TYPE               ((NMLTYPE) 207)
#define LBV_TRAJ_SET_MAX_ACCELERATION_TYPE           ((NMLTYPE) 208)
#define LBV_TRAJ_SET_SCALE_TYPE                      ((NMLTYPE) 209)
#define LBV_TRAJ_SET_RAPID_SCALE_TYPE                ((NMLTYPE) 238)
#define LBV_TRAJ_SET_MOTION_ID_TYPE                  ((NMLTYPE) 210)

#define LBV_TRAJ_INIT_TYPE                           ((NMLTYPE) 211)
#define LBV_TRAJ_HALT_TYPE                           ((NMLTYPE) 212)
#define LBV_TRAJ_ENABLE_TYPE                         ((NMLTYPE) 213)
#define LBV_TRAJ_DISABLE_TYPE                        ((NMLTYPE) 214)
#define LBV_TRAJ_ABORT_TYPE                          ((NMLTYPE) 215)
#define LBV_TRAJ_PAUSE_TYPE                          ((NMLTYPE) 216)
#define LBV_TRAJ_STEP_TYPE                           ((NMLTYPE) 217)
#define LBV_TRAJ_RESUME_TYPE                         ((NMLTYPE) 218)
#define LBV_TRAJ_DELAY_TYPE                          ((NMLTYPE) 219)
#define LBV_TRAJ_LINEAR_MOVE_TYPE                    ((NMLTYPE) 220)
#define LBV_TRAJ_CIRCULAR_MOVE_TYPE                  ((NMLTYPE) 221)
#define LBV_TRAJ_SET_TERM_COND_TYPE                  ((NMLTYPE) 222)
#define LBV_TRAJ_SET_OFFSET_TYPE                     ((NMLTYPE) 223)
#define LBV_TRAJ_SET_G5X_TYPE                        ((NMLTYPE) 224)
#define LBV_TRAJ_SET_HOME_TYPE                       ((NMLTYPE) 225)
#define LBV_TRAJ_SET_ROTATION_TYPE                   ((NMLTYPE) 226)
#define LBV_TRAJ_SET_G92_TYPE                        ((NMLTYPE) 227)
#define LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE       ((NMLTYPE) 228)
#define LBV_TRAJ_PROBE_TYPE                          ((NMLTYPE) 229)
#define LBV_TRAJ_SET_TELEOP_ENABLE_TYPE              ((NMLTYPE) 230)
#define LBV_TRAJ_SET_SPINDLESYNC_TYPE                ((NMLTYPE) 232)
#define LBV_TRAJ_SET_SPINDLE_SCALE_TYPE              ((NMLTYPE) 233)
#define LBV_TRAJ_SET_FO_ENABLE_TYPE                  ((NMLTYPE) 234)
#define LBV_TRAJ_SET_SO_ENABLE_TYPE                  ((NMLTYPE) 235)
#define LBV_TRAJ_SET_FH_ENABLE_TYPE                  ((NMLTYPE) 236)
#define LBV_TRAJ_RIGID_TAP_TYPE                      ((NMLTYPE) 237)

#define LBV_TRAJ_STAT_TYPE                           ((NMLTYPE) 299)

// LBV_MOTION aggregate class type declaration

#define LBV_MOTION_INIT_TYPE                         ((NMLTYPE) 301)
#define LBV_MOTION_HALT_TYPE                         ((NMLTYPE) 302)
#define LBV_MOTION_ABORT_TYPE                        ((NMLTYPE) 303)
#define LBV_MOTION_SET_AOUT_TYPE                     ((NMLTYPE) 304)
#define LBV_MOTION_SET_DOUT_TYPE                     ((NMLTYPE) 305)
#define LBV_MOTION_ADAPTIVE_TYPE                     ((NMLTYPE) 306)

#define LBV_MOTION_STAT_TYPE                         ((NMLTYPE) 399)

// NML for LBV_TASK

#define LBV_TASK_INIT_TYPE                           ((NMLTYPE) 501)
#define LBV_TASK_HALT_TYPE                           ((NMLTYPE) 502)
#define LBV_TASK_ABORT_TYPE                          ((NMLTYPE) 503)
#define LBV_TASK_SET_MODE_TYPE                       ((NMLTYPE) 504)
#define LBV_TASK_SET_STATE_TYPE                      ((NMLTYPE) 505)
#define LBV_TASK_PLAN_OPEN_TYPE                      ((NMLTYPE) 506)
#define LBV_TASK_PLAN_RUN_TYPE                       ((NMLTYPE) 507)
#define LBV_TASK_PLAN_READ_TYPE                      ((NMLTYPE) 508)
#define LBV_TASK_PLAN_EXECUTE_TYPE                   ((NMLTYPE) 509)
#define LBV_TASK_PLAN_PAUSE_TYPE                     ((NMLTYPE) 510)
#define LBV_TASK_PLAN_STEP_TYPE                      ((NMLTYPE) 511)
#define LBV_TASK_PLAN_RESUME_TYPE                    ((NMLTYPE) 512)
#define LBV_TASK_PLAN_END_TYPE                       ((NMLTYPE) 513)
#define LBV_TASK_PLAN_CLOSE_TYPE                     ((NMLTYPE) 514)
#define LBV_TASK_PLAN_INIT_TYPE                      ((NMLTYPE) 515)
#define LBV_TASK_PLAN_SYNCH_TYPE                     ((NMLTYPE) 516)
#define LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE         ((NMLTYPE) 517)
#define LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE          ((NMLTYPE) 518)
#define LBV_TASK_PLAN_OPTIONAL_STOP_TYPE             ((NMLTYPE) 519)
#define LBV_TASK_PLAN_REVERSE_TYPE                   ((NMLTYPE) 520)
#define LBV_TASK_PLAN_FORWARD_TYPE                   ((NMLTYPE) 521)

#define LBV_TASK_STAT_TYPE                           ((NMLTYPE) 599)

// LBV_TOOL type declarations

#define LBV_TOOL_INIT_TYPE                           ((NMLTYPE) 1101)
#define LBV_TOOL_HALT_TYPE                           ((NMLTYPE) 1102)
#define LBV_TOOL_ABORT_TYPE                          ((NMLTYPE) 1103)
#define LBV_TOOL_PREPARE_TYPE                        ((NMLTYPE) 1104)
#define LBV_TOOL_LOAD_TYPE                           ((NMLTYPE) 1105)
#define LBV_TOOL_UNLOAD_TYPE                         ((NMLTYPE) 1106)
#define LBV_TOOL_LOAD_TOOL_TABLE_TYPE                ((NMLTYPE) 1107)
#define LBV_TOOL_SET_OFFSET_TYPE                     ((NMLTYPE) 1108)
#define LBV_TOOL_SET_NUMBER_TYPE                     ((NMLTYPE) 1109)
// the following message is sent to io at the very start of an M6
// even before lbvcanon issues the move to toolchange position
#define LBV_TOOL_START_CHANGE_TYPE                   ((NMLTYPE) 1110)

#define LBV_EXEC_PLUGIN_CALL_TYPE                   ((NMLTYPE) 1112)
#define LBV_IO_PLUGIN_CALL_TYPE                   ((NMLTYPE) 1113)
#define LBV_TOOL_STAT_TYPE                           ((NMLTYPE) 1199)

// LBV_AUX type declarations
#define LBV_AUX_ESTOP_ON_TYPE                         ((NMLTYPE) 1206)
#define LBV_AUX_ESTOP_OFF_TYPE                        ((NMLTYPE) 1207)
#define LBV_AUX_ESTOP_RESET_TYPE                      ((NMLTYPE) 1208)
#define LBV_AUX_INPUT_WAIT_TYPE                       ((NMLTYPE) 1209)

#define LBV_AUX_STAT_TYPE                             ((NMLTYPE) 1299)

// LBV_SPINDLE type declarations
#define LBV_SPINDLE_ON_TYPE                          ((NMLTYPE) 1304)
#define LBV_SPINDLE_OFF_TYPE                         ((NMLTYPE) 1305)
#define LBV_SPINDLE_INCREASE_TYPE                    ((NMLTYPE) 1309)
#define LBV_SPINDLE_DECREASE_TYPE                    ((NMLTYPE) 1310)
#define LBV_SPINDLE_CONSTANT_TYPE                    ((NMLTYPE) 1311)
#define LBV_SPINDLE_BRAKE_RELEASE_TYPE               ((NMLTYPE) 1312)
#define LBV_SPINDLE_BRAKE_ENGAGE_TYPE                ((NMLTYPE) 1313)
#define LBV_SPINDLE_SPEED_TYPE                       ((NMLTYPE) 1316)
#define LBV_SPINDLE_ORIENT_TYPE                      ((NMLTYPE) 1317)
#define LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE        ((NMLTYPE) 1318)

#define LBV_SPINDLE_STAT_TYPE                        ((NMLTYPE) 1399)

// LBV_COOLANT type declarations
#define LBV_COOLANT_MIST_ON_TYPE                     ((NMLTYPE) 1404)
#define LBV_COOLANT_MIST_OFF_TYPE                    ((NMLTYPE) 1405)
#define LBV_COOLANT_FLOOD_ON_TYPE                    ((NMLTYPE) 1406)
#define LBV_COOLANT_FLOOD_OFF_TYPE                   ((NMLTYPE) 1407)

#define LBV_COOLANT_STAT_TYPE                        ((NMLTYPE) 1499)

// LBV_LUBE type declarations
#define LBV_LUBE_ON_TYPE                             ((NMLTYPE) 1504)
#define LBV_LUBE_OFF_TYPE                            ((NMLTYPE) 1505)
#define LBV_LUBE_STAT_TYPE                           ((NMLTYPE) 1599)

// LBV_IO aggregate class type declaration
#define LBV_IO_INIT_TYPE                             ((NMLTYPE) 1601)
#define LBV_IO_HALT_TYPE                             ((NMLTYPE) 1602)
#define LBV_IO_ABORT_TYPE                            ((NMLTYPE) 1603)
#define LBV_IO_SET_CYCLE_TIME_TYPE                   ((NMLTYPE) 1604)

#define LBV_IO_STAT_TYPE                             ((NMLTYPE) 1699)

// LBV aggregate class type declaration
// these are placeholders
#define LBV_LOG_TYPE_IO_CMD      21	// command into LBV IO controller
#define LBV_LOG_TYPE_TASK_CMD    51	// command into LBV Task controller

#define LBV_INIT_TYPE                                ((NMLTYPE) 1901)
#define LBV_HALT_TYPE                                ((NMLTYPE) 1902)
#define LBV_ABORT_TYPE                               ((NMLTYPE) 1903)

#define LBV_STAT_TYPE                                ((NMLTYPE) 1999)

// types for LBV_TASK mode
enum LBV_TASK_MODE_ENUM {
    LBV_TASK_MODE_MANUAL = 1,
    LBV_TASK_MODE_AUTO = 2,
    LBV_TASK_MODE_MDI = 3
};

// types for LBV_TASK state
enum LBV_TASK_STATE_ENUM {
    LBV_TASK_STATE_ESTOP = 1,
    LBV_TASK_STATE_ESTOP_RESET = 2,
    LBV_TASK_STATE_OFF = 3,
    LBV_TASK_STATE_ON = 4
};

// types for LBV_TASK execState
enum LBV_TASK_EXEC_ENUM {
    LBV_TASK_EXEC_ERROR = 1,
    LBV_TASK_EXEC_DONE = 2,
    LBV_TASK_EXEC_WAITING_FOR_MOTION = 3,
    LBV_TASK_EXEC_WAITING_FOR_MOTION_QUEUE = 4,
    LBV_TASK_EXEC_WAITING_FOR_IO = 5,
    LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO = 7,
    LBV_TASK_EXEC_WAITING_FOR_DELAY = 8,
    LBV_TASK_EXEC_WAITING_FOR_SYSTEM_CMD = 9,
    LBV_TASK_EXEC_WAITING_FOR_SPINDLE_ORIENTED = 10
};

// types for LBV_TASK interpState
enum LBV_TASK_INTERP_ENUM {
    LBV_TASK_INTERP_IDLE = 1,
    LBV_TASK_INTERP_READING = 2,
    LBV_TASK_INTERP_PAUSED = 3,
    LBV_TASK_INTERP_WAITING = 4
};

// types for motion control
enum LBV_TRAJ_MODE_ENUM {
    LBV_TRAJ_MODE_FREE = 1,	// independent-axis motion,
    LBV_TRAJ_MODE_COORD = 2,	// coordinated-axis motion,
    LBV_TRAJ_MODE_TELEOP = 3	// velocity based world coordinates motion,
};

// types for lbvIoAbort() reasons
enum LBV_IO_ABORT_REASON_ENUM {
	LBV_ABORT_TASK_EXEC_ERROR = 1,
	LBV_ABORT_AUX_ESTOP = 2,
	LBV_ABORT_MOTION_OR_IO_RCS_ERROR = 3,
	LBV_ABORT_TASK_STATE_OFF = 4,
	LBV_ABORT_TASK_STATE_ESTOP_RESET = 5,
	LBV_ABORT_TASK_STATE_ESTOP = 6,
	LBV_ABORT_TASK_STATE_NOT_ON = 7,
	LBV_ABORT_TASK_ABORT = 8,
	LBV_ABORT_INTERPRETER_ERROR = 9,	// interpreter failed during readahead
	LBV_ABORT_INTERPRETER_ERROR_MDI = 10,	// interpreter failed during MDI execution
	LBV_ABORT_USER = 100  // user-defined abort codes start here
};
// --------------
// LBV VOCABULARY
// --------------

// NML formatting function
extern int lbvFormat(NMLTYPE type, void *buffer, CMS * cms);

// NML Symbol Lookup Function
extern const char *lbv_symbol_lookup(uint32_t type);
#define lbvSymbolLookup(a) lbv_symbol_lookup(a)

// decls for command line args-- mains are responsible for setting these
// so that other modules can get cmd line args for ad hoc processing
extern int Argc;
extern char **Argv;

// ------------------------
// IMPLEMENTATION FUNCTIONS
// ------------------------

// implementation functions for LBV error, message types
// intended to be implemented in main() file, by writing to NML buffer

// print an error
extern int lbvOperatorError(int id, const char *fmt, ...) __attribute__((format(printf,2,3)));

// print general text
extern int lbvOperatorText(int id, const char *fmt, ...) __attribute__((format(printf,2,3)));

// print note to operator
extern int lbvOperatorDisplay(int id, const char *fmt, ...) __attribute__((format(printf,2,3)));

// implementation functions for LBV_AXIS types

extern int lbvAxisSetUnits(int axis, double units);
extern int lbvAxisSetMinPositionLimit(int axis, double limit);
extern int lbvAxisSetMaxPositionLimit(int axis, double limit);
extern int lbvAxisSetMaxVelocity(int axis, double vel, double ext_offset_vel);
extern int lbvAxisSetMaxAcceleration(int axis, double acc, double ext_offset_acc);
extern double lbvAxisGetMaxVelocity(int axis);
extern double lbvAxisGetMaxAcceleration(int axis);
extern int lbvAxisSetLockingJoint(int axis,int joint);

extern int lbvAxisUpdate(LBV_AXIS_STAT stat[], int numAxes);

// implementation functions for LBV_JOINT types

extern int lbvJointSetType(int joint, unsigned char jointType);
extern int lbvJointSetUnits(int joint, double units);
extern int lbvJointSetBacklash(int joint, double backlash);
extern int lbvJointSetMinPositionLimit(int joint, double limit);
extern int lbvJointSetMaxPositionLimit(int joint, double limit);
extern int lbvJointSetMotorOffset(int joint, double offset);
extern int lbvJointSetFerror(int joint, double ferror);
extern int lbvJointSetMinFerror(int joint, double ferror);
extern int lbvJointSetHomingParams(int joint, double home, double offset, double home_vel,
				  double search_vel, double latch_vel,
				  int use_index, int encoder_does_not_reset, int ignore_limits,
				  int is_shared, int home_sequence, int volatile_home, int locking_indexer,
                  int absolute_encoder);
extern int lbvJointUpdateHomingParams(int joint, double home, double offset, int sequence);
extern int lbvJointSetMaxVelocity(int joint, double vel);
extern int lbvJointSetMaxAcceleration(int joint, double acc);

extern int lbvJointInit(int joint);
extern int lbvJointHalt(int joint);
extern int lbvJointEnable(int joint);
extern int lbvJointDisable(int joint);
extern int lbvJointHome(int joint);
extern int lbvJointUnhome(int joint);
extern int lbvJointActivate(int joint);
extern int lbvJointDeactivate(int joint);
extern int lbvJointOverrideLimits(int joint);
extern int lbvJointLoadComp(int joint, const char *file, int type);
extern int lbvJogStop(int nr, int jjogmode);
extern int lbvJogCont(int nr, double vel, int jjogmode);
extern int lbvJogIncr(int nr, double incr, double vel, int jjogmode);
extern int lbvJogAbs(int nr, double pos, double vel, int jjogmode);


extern int lbvJointUpdate(LBV_JOINT_STAT stat[], int numJoints);

// implementation functions for LBV_TRAJ types

extern int lbvTrajSetJoints(int joints);
extern int lbvTrajSetAxes(int axismask);
extern int lbvTrajSetSpindles(int spindles);
extern int lbvTrajSetUnits(double linearUnits, double angularUnits);
extern int lbvTrajSetCycleTime(double cycleTime);
extern int lbvTrajSetMode(int traj_mode);
extern int lbvTrajSetVelocity(double vel, double ini_maxvel);
extern int lbvTrajSetAcceleration(double acc);
extern int lbvTrajSetMaxVelocity(double vel);
extern int lbvTrajSetMaxAcceleration(double acc);
extern int lbvTrajSetScale(double scale);
extern int lbvTrajSetRapidScale(double scale);
extern int lbvTrajSetFOEnable(unsigned char mode);   //feed override enable
extern int lbvTrajSetFHEnable(unsigned char mode);   //feed hold enable
extern int lbvTrajSetSpindleScale(int spindle, double scale);
extern int lbvTrajSetSOEnable(unsigned char mode);   //spindle speed override enable
extern int lbvTrajSetAFEnable(unsigned char enable); //adaptive feed enable
extern int lbvTrajSetMotionId(int id);
extern double lbvTrajGetLinearUnits();
extern double lbvTrajGetAngularUnits();

extern int lbvTrajInit();
extern int lbvTrajHalt();
extern int lbvTrajEnable();
extern int lbvTrajDisable();
extern int lbvTrajAbort();
extern int lbvTrajPause();
extern int lbvTrajReverse();
extern int lbvTrajForward();
extern int lbvTrajStep();
extern int lbvTrajResume();
extern int lbvTrajDelay(double delay);
extern int lbvTrajLinearMove(LbvPose end, int type, double vel,
                             double ini_maxvel, double acc, int indexer_jnum);
extern int lbvTrajCircularMove(LbvPose end, PM_CARTESIAN center, PM_CARTESIAN
        normal, int turn, int type, double vel, double ini_maxvel, double acc);
extern int lbvTrajSetTermCond(int cond, double tolerance);
extern int lbvTrajSetSpindleSync(int spindle, double feed_per_revolution, bool wait_for_index);
extern int lbvTrajSetOffset(LbvPose tool_offset);
extern int lbvTrajSetOrigin(LbvPose origin);
extern int lbvTrajSetRotation(double rotation);
extern int lbvTrajSetHome(LbvPose home);
extern int lbvTrajClearProbeTrippedFlag();
extern int lbvTrajProbe(LbvPose pos, int type, double vel, 
                        double ini_maxvel, double acc, unsigned char probe_type);
extern int lbvAuxInputWait(int index, int input_type, int wait_type, int timeout);
extern int lbvTrajRigidTap(LbvPose pos, double vel, double ini_maxvel, double acc, double scale);

extern int lbvTrajUpdate(LBV_TRAJ_STAT * stat);

// implementation functions for LBV_MOTION aggregate types

extern int lbvMotionInit();
extern int lbvMotionHalt();
extern int lbvMotionAbort();
extern int lbvMotionSetDebug(int debug);
extern int lbvMotionSetAout(unsigned char index, double start, double end,
                            unsigned char now);
extern int lbvMotionSetDout(unsigned char index, unsigned char start,
			    unsigned char end, unsigned char now);

extern int lbvMotionUpdate(LBV_MOTION_STAT * stat);

extern int lbvAbortCleanup(int reason,const char *message = "");

int setup_inihal(void);

// implementation functions for LBV_TOOL types

extern int lbvToolInit();
extern int lbvToolHalt();
extern int lbvToolAbort();
extern int lbvToolPrepare(int pocket, int tool);
extern int lbvToolLoad();
extern int lbvToolUnload();
extern int lbvToolLoadToolTable(const char *file);
extern int lbvToolSetOffset(int pocket, int toolno, LbvPose offset, double diameter,
                            double frontangle, double backangle, int orientation);
extern int lbvToolSetNumber(int number);
extern int lbvToolStartChange();

extern int lbvToolSetToolTableFile(const char *file);

extern int lbvToolUpdate(LBV_TOOL_STAT * stat);

// implementation functions for LBV_AUX types

extern int lbvAuxEstopOn();
extern int lbvAuxEstopOff();

extern int lbvAuxUpdate(LBV_AUX_STAT * stat);

// implementation functions for LBV_SPINDLE types

extern int lbvSpindleAbort(int spindle);
extern int lbvSpindleSpeed(int spindle, double speed, double factor, double xoffset);
extern int lbvSpindleOn(int spindle, double speed, double factor, double xoffset,int wait_for_atspeed = 1);
extern int lbvSpindleOrient(int spindle, double orientation, int direction);
extern int lbvSpindleWaitOrientComplete(double timout);
extern int lbvSpindleOff(int spindle);
extern int lbvSpindleIncrease(int spindle);
extern int lbvSpindleDecrease(int spindle);
extern int lbvSpindleConstant(int spindle);
extern int lbvSpindleBrakeRelease(int spindle);
extern int lbvSpindleBrakeEngage(int spindle);

extern int lbvSpindleSetMode(int mode); //determines if Spindle needs to reset on abort

extern int lbvSpindleUpdate(LBV_SPINDLE_STAT stat[], int num_spindles);

// implementation functions for LBV_COOLANT types

extern int lbvCoolantMistOn();
extern int lbvCoolantMistOff();
extern int lbvCoolantFloodOn();
extern int lbvCoolantFloodOff();

extern int lbvCoolantUpdate(LBV_COOLANT_STAT * stat);

// implementation functions for LBV_LUBE types

extern int lbvLubeOn();
extern int lbvLubeOff();

extern int lbvLubeUpdate(LBV_LUBE_STAT * stat);

// implementation functions for LBV_IO types

extern int lbvIoInit();
extern int lbvIoHalt();
extern int lbvIoAbort(int reason);
extern int lbvIoSetCycleTime(double cycleTime);
extern int lbvIoSetDebug(int debug);

extern int lbvIoUpdate(LBV_IO_STAT * stat);

// implementation functions for LBV aggregate types

extern int lbvInit();
extern int lbvHalt();
extern int lbvAbort();

int lbvSetMaxFeedOverride(double maxFeedScale);
int lbvSetupArcBlends(int arcBlendEnable,
        int arcBlendFallbackEnable,
        int arcBlendOptDepth,
        int arcBlendGapCycles,
        double arcBlendRampFreq,
        double arcBlendTangentKinkRatio);
int lbvSetProbeErrorInhibit(int j_inhibit, int h_inhibit);
int lbvGetExternalOffsetApplied(void);
LbvPose lbvGetExternalOffsets(void);

extern int lbvUpdate(LBV_STAT * stat);
// full LBV status
extern LBV_STAT *lbvStatus;

// LBV IO status
extern LBV_IO_STAT *lbvIoStatus;

// LBV MOTION status
extern LBV_MOTION_STAT *lbvMotionStatus;

// values for LBV_JOINT_SET_JOINT, jointType
enum LbvJointType {
    LBV_LINEAR             = 1,
    LBV_ANGULAR            = 2,
};

/**
 * Set the units conversion factor.
 * @see LBV_JOINT_SET_INPUT_SCALE
 */
typedef double                  LbvLinearUnits;
typedef double                  LbvAngularUnits;

#endif				// #ifndef LBV_HH
