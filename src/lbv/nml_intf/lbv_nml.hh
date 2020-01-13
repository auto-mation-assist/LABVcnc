/********************************************************************
* Description: lbv_nml.hh
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
#ifndef LBV_NML_HH
#define LBV_NML_HH
#include "lbv.hh"
#include "rcs.hh"
#include "cmd_msg.hh"
#include "stat_msg.hh"
#include "lbvpos.h"
#include "canon.hh"		// CANON_TOOL_TABLE, CANON_UNITS
#include "rs274ngc.hh"		// ACTIVE_G_CODES, etc

// ------------------
// CLASS DECLARATIONS
// ------------------

// declarations for LBV general classes

/**
 * Send a textual error message to the operator.
 * The message is put in the errlog buffer to be read by the GUI.
 * This allows the controller a generic way to send error messages to
 * the operator.
 */
class LBV_OPERATOR_ERROR:public RCS_CMD_MSG {
  public:
    LBV_OPERATOR_ERROR():RCS_CMD_MSG(LBV_OPERATOR_ERROR_TYPE,
				     sizeof(LBV_OPERATOR_ERROR)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int id;
    char error[LINELEN];
};

/**
 * Send a textual information message to the operator.
 * This is similiar to LBV_OPERATOR_ERROR message except that the messages are
 * sent in situations not necessarily considered to be errors.
 */
class LBV_OPERATOR_TEXT:public RCS_CMD_MSG {
  public:
    LBV_OPERATOR_TEXT():RCS_CMD_MSG(LBV_OPERATOR_TEXT_TYPE,
				    sizeof(LBV_OPERATOR_TEXT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int id;
    char text[LINELEN];
};

/**
 * Send the URL or filename of a document to display.
 * This message is placed in the errlog buffer  to be read by the GUI.
 * If the GUI is capable of doing so it will show the operator a
 * previously created document, using the URL or filename provided.
 * This message is placed in the errlog channel to be read by the GUI.
 * This provides a general means of reporting an error from within the
 * controller without having to program the GUI to recognize each error type.
 */
class LBV_OPERATOR_DISPLAY:public RCS_CMD_MSG {
  public:
    LBV_OPERATOR_DISPLAY():RCS_CMD_MSG(LBV_OPERATOR_DISPLAY_TYPE,
				       sizeof(LBV_OPERATOR_DISPLAY)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int id;
    char display[LINELEN];
};

#define LBV_SYSTEM_CMD_LEN 256
/*
  execute a system command
*/
class LBV_SYSTEM_CMD:public RCS_CMD_MSG {
  public:
    LBV_SYSTEM_CMD():RCS_CMD_MSG(LBV_SYSTEM_CMD_TYPE,
				 sizeof(LBV_SYSTEM_CMD)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    char string[LBV_SYSTEM_CMD_LEN];
};

class LBV_NULL:public RCS_CMD_MSG {
  public:
    LBV_NULL():RCS_CMD_MSG(LBV_NULL_TYPE, sizeof(LBV_NULL)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_SET_DEBUG:public RCS_CMD_MSG {
  public:
    LBV_SET_DEBUG():RCS_CMD_MSG(LBV_SET_DEBUG_TYPE, sizeof(LBV_SET_DEBUG)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int debug;
};


/*
 * LBV_JOG_CMD_MSG class.
 */
class LBV_JOG_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_JOG_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // joint_or_axis == joint_number          for joint jogs (jjogmode==1)
    // joint_or_axis == 0 for X, 1 for Y,...  for axis  jogs (jjogmode==0)
    int joint_or_axis;
};

// AXIS status base class
class LBV_AXIS_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_AXIS_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int axis;
};

class LBV_AXIS_STAT:public LBV_AXIS_STAT_MSG {
  public:
    LBV_AXIS_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double minPositionLimit;
    double maxPositionLimit;
    double velocity;		// current velocity
};

// declarations for LBV_JOINT classes

/*
 * JOINT command base class.
 * This is the base class for all commands that operate on a single joint.
 * The joint parameter specifies which joint the command affects.
 * These commands are sent to the lbvCommand buffer to be read by the
 * TASK program that will then pass along corresponding messages to the
 * motion system.
 */
class LBV_JOINT_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_JOINT_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int joint;
};

/**
 * Set the joint type to linear or angular.
 * Similiar to the JOINT_TYPE field in the ".ini" file.
 */
class LBV_JOINT_SET_JOINT:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_JOINT():LBV_JOINT_CMD_MSG(LBV_JOINT_SET_JOINT_TYPE,
					 sizeof(LBV_JOINT_SET_JOINT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // LBV_JOINT_LINEAR, LBV_JOINT_ANGULAR
    unsigned char jointType;
};

/**
 * Set the units conversion factor.
 * @see LBV_JOINT_SET_INPUT_SCALE
 */
class LBV_JOINT_SET_UNITS:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_UNITS():LBV_JOINT_CMD_MSG(LBV_JOINT_SET_UNITS_TYPE,
					  sizeof(LBV_JOINT_SET_UNITS)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // units per mm, deg for linear, angular
    double units;
};

/**
 * Set the Axis backlash.
 * This command sets the backlash value.
 */
class LBV_JOINT_SET_BACKLASH:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_BACKLASH():LBV_JOINT_CMD_MSG(LBV_JOINT_SET_BACKLASH_TYPE,
					     sizeof(LBV_JOINT_SET_BACKLASH))
    {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double backlash;
};

class LBV_JOINT_SET_MIN_POSITION_LIMIT:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_MIN_POSITION_LIMIT():LBV_JOINT_CMD_MSG
	(LBV_JOINT_SET_MIN_POSITION_LIMIT_TYPE,
	 sizeof(LBV_JOINT_SET_MIN_POSITION_LIMIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double limit;
};

class LBV_JOINT_SET_MAX_POSITION_LIMIT:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_MAX_POSITION_LIMIT():LBV_JOINT_CMD_MSG
	(LBV_JOINT_SET_MAX_POSITION_LIMIT_TYPE,
	 sizeof(LBV_JOINT_SET_MAX_POSITION_LIMIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double limit;
};

class LBV_JOINT_SET_FERROR:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_FERROR():LBV_JOINT_CMD_MSG(LBV_JOINT_SET_FERROR_TYPE,
					   sizeof(LBV_JOINT_SET_FERROR)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double ferror;
};

class LBV_JOINT_SET_MIN_FERROR:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_MIN_FERROR():LBV_JOINT_CMD_MSG
	(LBV_JOINT_SET_MIN_FERROR_TYPE, sizeof(LBV_JOINT_SET_MIN_FERROR)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double ferror;
};

class LBV_JOINT_SET_HOMING_PARAMS:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_HOMING_PARAMS():LBV_JOINT_CMD_MSG
	(LBV_JOINT_SET_HOMING_PARAMS_TYPE,
	 sizeof(LBV_JOINT_SET_HOMING_PARAMS)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double home;
    double offset;
    double home_final_vel;
    double search_vel;
    double latch_vel;
    int use_index;
    int encoder_does_not_reset;
    int ignore_limits;
    int is_shared;
    int home_sequence;
    int volatile_home;
    int locking_indexer;
    int absolute_encoder;
};

class LBV_JOINT_SET_MAX_VELOCITY:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_SET_MAX_VELOCITY():LBV_JOINT_CMD_MSG
	(LBV_JOINT_SET_MAX_VELOCITY_TYPE,
	 sizeof(LBV_JOINT_SET_MAX_VELOCITY)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double vel;
};

class LBV_JOINT_INIT:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_INIT():LBV_JOINT_CMD_MSG(LBV_JOINT_INIT_TYPE,
				     sizeof(LBV_JOINT_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_HALT:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_HALT():LBV_JOINT_CMD_MSG(LBV_JOINT_HALT_TYPE,
				     sizeof(LBV_JOINT_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_ABORT:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_ABORT():LBV_JOINT_CMD_MSG(LBV_JOINT_ABORT_TYPE,
				      sizeof(LBV_JOINT_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_ENABLE:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_ENABLE():LBV_JOINT_CMD_MSG(LBV_JOINT_ENABLE_TYPE,
				       sizeof(LBV_JOINT_ENABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_DISABLE:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_DISABLE():LBV_JOINT_CMD_MSG(LBV_JOINT_DISABLE_TYPE,
					sizeof(LBV_JOINT_DISABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_HOME:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_HOME():LBV_JOINT_CMD_MSG(LBV_JOINT_HOME_TYPE,
				     sizeof(LBV_JOINT_HOME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_UNHOME:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_UNHOME():LBV_JOINT_CMD_MSG(LBV_JOINT_UNHOME_TYPE,
				     sizeof(LBV_JOINT_UNHOME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOG_CONT:public LBV_JOG_CMD_MSG {
  public:
    LBV_JOG_CONT():LBV_JOG_CMD_MSG(LBV_JOG_CONT_TYPE,
				    sizeof(LBV_JOG_CONT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double vel;
    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class LBV_JOG_INCR:public LBV_JOG_CMD_MSG {
  public:
    LBV_JOG_INCR():LBV_JOG_CMD_MSG(LBV_JOG_INCR_TYPE,
					 sizeof(LBV_JOG_INCR)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double incr;
    double vel;
    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class LBV_JOG_ABS:public LBV_JOG_CMD_MSG {
  public:
    LBV_JOG_ABS():LBV_JOG_CMD_MSG(LBV_JOG_ABS_TYPE,
					sizeof(LBV_JOG_ABS)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double pos;
    double vel;
    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class LBV_JOG_STOP:public LBV_JOG_CMD_MSG {
  public:
    LBV_JOG_STOP():LBV_JOG_CMD_MSG(LBV_JOG_STOP_TYPE,
				    sizeof(LBV_JOG_STOP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int jjogmode; // 1==> joint jog, 0==> axis jog
};

class LBV_JOINT_ACTIVATE:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_ACTIVATE():LBV_JOINT_CMD_MSG(LBV_JOINT_ACTIVATE_TYPE,
					 sizeof(LBV_JOINT_ACTIVATE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_DEACTIVATE:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_DEACTIVATE():LBV_JOINT_CMD_MSG(LBV_JOINT_DEACTIVATE_TYPE,
					   sizeof(LBV_JOINT_DEACTIVATE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_OVERRIDE_LIMITS:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_OVERRIDE_LIMITS():LBV_JOINT_CMD_MSG
	(LBV_JOINT_OVERRIDE_LIMITS_TYPE, sizeof(LBV_JOINT_OVERRIDE_LIMITS)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_JOINT_LOAD_COMP:public LBV_JOINT_CMD_MSG {
  public:
    LBV_JOINT_LOAD_COMP():LBV_JOINT_CMD_MSG(LBV_JOINT_LOAD_COMP_TYPE,
					  sizeof(LBV_JOINT_LOAD_COMP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    char file[LINELEN];
    int type; // type of the comp file. type==0 means nom, forw, rev triplets
              // type != 0 means nom, forw_trim, rev_trim triplets
};


// JOINT status base class
class LBV_JOINT_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_JOINT_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int joint;
};

class LBV_JOINT_STAT:public LBV_JOINT_STAT_MSG {
  public:
    LBV_JOINT_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // configuration parameters
    unsigned char jointType;	// LBV_JOINT_LINEAR, LBV_JOINT_ANGULAR
    double units;		// units per mm, deg for linear, angular
    double backlash;
    double minPositionLimit;
    double maxPositionLimit;
    double maxFerror;
    double minFerror;

    // dynamic status
    double ferrorCurrent;	// current following error
    double ferrorHighMark;	// magnitude of max following error
    /*! \todo FIXME - is this really position, or the DAC output? */
    double output;		// commanded output position
    double input;		// current input position
    double velocity;		// current velocity
    unsigned char inpos;	// non-zero means in position
    unsigned char homing;	// non-zero means homing
    unsigned char homed;	// non-zero means has been homed
    unsigned char fault;	// non-zero means axis amp fault
    unsigned char enabled;	// non-zero means enabled
    unsigned char minSoftLimit;	// non-zero means min soft limit exceeded
    unsigned char maxSoftLimit;	// non-zero means max soft limit exceeded
    unsigned char minHardLimit;	// non-zero means min hard limit exceeded
    unsigned char maxHardLimit;	// non-zero means max hard limit exceeded
    unsigned char overrideLimits; // non-zero means limits are overridden
};

// declarations for LBV_TRAJ classes

// LBV_TRAJ command base class
class LBV_TRAJ_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_TRAJ_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_SET_UNITS:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_UNITS():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_UNITS_TYPE,
					  sizeof(LBV_TRAJ_SET_UNITS)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double linearUnits;		// units per mm
    double angularUnits;	// units per degree
};

class LBV_TRAJ_SET_AXES:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_AXES():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_AXES_TYPE,
					 sizeof(LBV_TRAJ_SET_AXES)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int axes;
};

class LBV_TRAJ_SET_CYCLE_TIME:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_CYCLE_TIME():LBV_TRAJ_CMD_MSG
	(LBV_TRAJ_SET_CYCLE_TIME_TYPE, sizeof(LBV_TRAJ_SET_CYCLE_TIME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double cycleTime;
};

class LBV_TRAJ_SET_MODE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_MODE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_MODE_TYPE,
					 sizeof(LBV_TRAJ_SET_MODE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    enum LBV_TRAJ_MODE_ENUM mode;
};

class LBV_TRAJ_SET_VELOCITY:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_VELOCITY():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_VELOCITY_TYPE,
					     sizeof(LBV_TRAJ_SET_VELOCITY))
    {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double velocity;
    double ini_maxvel;
};

class LBV_TRAJ_SET_ACCELERATION:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_ACCELERATION():LBV_TRAJ_CMD_MSG
	(LBV_TRAJ_SET_ACCELERATION_TYPE,
	 sizeof(LBV_TRAJ_SET_ACCELERATION)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double acceleration;
};

class LBV_TRAJ_SET_MAX_VELOCITY:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_MAX_VELOCITY():LBV_TRAJ_CMD_MSG
	(LBV_TRAJ_SET_MAX_VELOCITY_TYPE,
	 sizeof(LBV_TRAJ_SET_MAX_VELOCITY)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double velocity;
};

class LBV_TRAJ_SET_MAX_ACCELERATION:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_MAX_ACCELERATION():LBV_TRAJ_CMD_MSG
	(LBV_TRAJ_SET_MAX_ACCELERATION_TYPE,
	 sizeof(LBV_TRAJ_SET_MAX_ACCELERATION)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double acceleration;
};

class LBV_TRAJ_SET_SCALE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_SCALE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_SCALE_TYPE,
					  sizeof(LBV_TRAJ_SET_SCALE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double scale;
};

class LBV_TRAJ_SET_RAPID_SCALE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_RAPID_SCALE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_RAPID_SCALE_TYPE,
					  sizeof(LBV_TRAJ_SET_RAPID_SCALE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double scale;
};

class LBV_TRAJ_SET_SPINDLE_SCALE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_SPINDLE_SCALE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_SPINDLE_SCALE_TYPE,
					  sizeof(LBV_TRAJ_SET_SPINDLE_SCALE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int spindle;
    double scale;
};

class LBV_TRAJ_SET_FO_ENABLE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_FO_ENABLE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_FO_ENABLE_TYPE,
					  sizeof(LBV_TRAJ_SET_FO_ENABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    unsigned char mode; //mode=0, override off (will work with 100% FO), mode != 0, override on, user can change FO
};

class LBV_TRAJ_SET_SO_ENABLE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_SO_ENABLE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_SO_ENABLE_TYPE,
					  sizeof(LBV_TRAJ_SET_SO_ENABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
    unsigned char mode; //mode=0, override off (will work with 100% SO), mode != 0, override on, user can change SO
};

class LBV_TRAJ_SET_FH_ENABLE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_FH_ENABLE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_FH_ENABLE_TYPE,
					  sizeof(LBV_TRAJ_SET_FH_ENABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    unsigned char mode; //mode=0, override off (feedhold is disabled), mode != 0, override on, user can use feedhold
};

class LBV_TRAJ_SET_MOTION_ID:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_MOTION_ID():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_MOTION_ID_TYPE,
					      sizeof
					      (LBV_TRAJ_SET_MOTION_ID)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int id;
};

class LBV_TRAJ_INIT:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_INIT():LBV_TRAJ_CMD_MSG(LBV_TRAJ_INIT_TYPE,
				     sizeof(LBV_TRAJ_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_HALT:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_HALT():LBV_TRAJ_CMD_MSG(LBV_TRAJ_HALT_TYPE,
				     sizeof(LBV_TRAJ_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_ENABLE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_ENABLE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_ENABLE_TYPE,
				       sizeof(LBV_TRAJ_ENABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_DISABLE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_DISABLE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_DISABLE_TYPE,
					sizeof(LBV_TRAJ_DISABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_ABORT:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_ABORT():LBV_TRAJ_CMD_MSG(LBV_TRAJ_ABORT_TYPE,
				      sizeof(LBV_TRAJ_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_PAUSE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_PAUSE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_PAUSE_TYPE,
				      sizeof(LBV_TRAJ_PAUSE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_STEP:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_STEP():LBV_TRAJ_CMD_MSG(LBV_TRAJ_STEP_TYPE,
				     sizeof(LBV_TRAJ_STEP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_RESUME:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_RESUME():LBV_TRAJ_CMD_MSG(LBV_TRAJ_RESUME_TYPE,
				       sizeof(LBV_TRAJ_RESUME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_DELAY:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_DELAY():LBV_TRAJ_CMD_MSG(LBV_TRAJ_DELAY_TYPE,
				      sizeof(LBV_TRAJ_DELAY)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double delay;		// delay in seconds
};

class LBV_TRAJ_LINEAR_MOVE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_LINEAR_MOVE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_LINEAR_MOVE_TYPE,
					    sizeof(LBV_TRAJ_LINEAR_MOVE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int type;
    LbvPose end;		// end point
    double vel, ini_maxvel, acc;
    int feed_mode;
    int indexer_jnum;
};

class LBV_TRAJ_CIRCULAR_MOVE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_CIRCULAR_MOVE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_CIRCULAR_MOVE_TYPE,
					      sizeof
					      (LBV_TRAJ_CIRCULAR_MOVE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    LbvPose end;
    PM_CARTESIAN center;
    PM_CARTESIAN normal;
    int turn;
    int type;
    double vel, ini_maxvel, acc;
    int feed_mode;
};

class LBV_TRAJ_SET_TERM_COND:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_TERM_COND():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_TERM_COND_TYPE,
					      sizeof
					      (LBV_TRAJ_SET_TERM_COND)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int cond;
    double tolerance; // used to set the precision/tolerance of path deviation 
		      // during CONTINUOUS motion mode. 
};

class LBV_TRAJ_SET_SPINDLESYNC:public LBV_TRAJ_CMD_MSG {
    public:
        LBV_TRAJ_SET_SPINDLESYNC():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_SPINDLESYNC_TYPE,
                sizeof(LBV_TRAJ_SET_SPINDLESYNC)) {
        };

        void update(CMS * cms);

        int spindle;
        double feed_per_revolution;
	bool velocity_mode; 
};

class LBV_TRAJ_SET_OFFSET:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_OFFSET():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_OFFSET_TYPE,
					   sizeof(LBV_TRAJ_SET_OFFSET)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    LbvPose offset;
};

class LBV_TRAJ_SET_G5X:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_G5X():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_G5X_TYPE,
					   sizeof(LBV_TRAJ_SET_G5X)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int g5x_index;
    LbvPose origin;
};

class LBV_TRAJ_SET_G92:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_G92():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_G92_TYPE,
					   sizeof(LBV_TRAJ_SET_G92)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    LbvPose origin;
};

class LBV_TRAJ_SET_ROTATION:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_ROTATION():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_ROTATION_TYPE,
					   sizeof(LBV_TRAJ_SET_ROTATION)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double rotation;
};

class LBV_TRAJ_SET_HOME:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_HOME():LBV_TRAJ_CMD_MSG(LBV_TRAJ_SET_HOME_TYPE,
					 sizeof(LBV_TRAJ_SET_HOME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    LbvPose home;
};

class LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG():LBV_TRAJ_CMD_MSG
	(LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE,
	 sizeof(LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_SET_TELEOP_ENABLE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_SET_TELEOP_ENABLE():LBV_TRAJ_CMD_MSG
	(LBV_TRAJ_SET_TELEOP_ENABLE_TYPE,
	 sizeof(LBV_TRAJ_SET_TELEOP_ENABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int enable;
};

class LBV_TRAJ_PROBE:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_PROBE():LBV_TRAJ_CMD_MSG(LBV_TRAJ_PROBE_TYPE,
				      sizeof(LBV_TRAJ_PROBE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    LbvPose pos;
    int type;
    double vel, ini_maxvel, acc;
    unsigned char probe_type;
};

class LBV_TRAJ_RIGID_TAP:public LBV_TRAJ_CMD_MSG {
  public:
    LBV_TRAJ_RIGID_TAP():LBV_TRAJ_CMD_MSG(LBV_TRAJ_RIGID_TAP_TYPE,
				      sizeof(LBV_TRAJ_RIGID_TAP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    LbvPose pos;
    double vel, ini_maxvel, acc, scale;
};

// LBV_TRAJ status base class
class LBV_TRAJ_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_TRAJ_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TRAJ_STAT:public LBV_TRAJ_STAT_MSG {
  public:
    LBV_TRAJ_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double linearUnits;		// units per mm
    double angularUnits;	// units per degree
    double cycleTime;		// cycle time, in seconds
    int joints;			// maximum joint number
    int spindles;			// maximum spindle number
    union {
        int deprecated_axes;
        int axes __attribute__((deprecated));			// maximum axis number
    };
    int axis_mask;		// mask of axes actually present
    enum LBV_TRAJ_MODE_ENUM mode;	// LBV_TRAJ_MODE_FREE,
    // LBV_TRAJ_MODE_COORD
    bool enabled;		// non-zero means enabled

    bool inpos;			// non-zero means in position
    int queue;			// number of pending motions, counting
    // current
    int activeQueue;		// number of motions blending
    bool queueFull;		// non-zero means can't accept another motion
    int id;			// id of the currently executing motion
    bool paused;			// non-zero means motion paused
    double scale;		// velocity scale factor
    double rapid_scale;		// rapid scale factor
    //double spindle_scale;	// moved to LBV_SPINDLE_STAT

    LbvPose position;		// current commanded position
    LbvPose actualPosition;	// current actual position, from forward kins
    double velocity;		// system velocity, for subsequent motions
    double acceleration;	// system acceleration, for subsequent
    // motions
    double maxVelocity;		// max system velocity
    double maxAcceleration;	// system acceleration

    LbvPose probedPosition;	// last position where probe was tripped.
    bool probe_tripped;		// Has the probe been tripped since the last
    // clear.
    bool probing;		// Are we currently looking for a probe
    // signal.
    int probeval;		// Current value of probe input.
    int kinematics_type;	// identity=1,serial=2,parallel=3,custom=4
    int motion_type;
    double distance_to_go;         // in current move
    LbvPose dtg;
    double current_vel;         // in current move
    bool feed_override_enabled;
    //bool spindle_override_enabled; moved to SPINDLE_STAT
    bool adaptive_feed_enabled;
    bool feed_hold_enabled;
};

// lbv_MOTION is aggregate of all LBV motion-related status classes

// LBV_MOTION command base class
class LBV_MOTION_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_MOTION_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_MOTION_INIT:public LBV_MOTION_CMD_MSG {
  public:
    LBV_MOTION_INIT():LBV_MOTION_CMD_MSG(LBV_MOTION_INIT_TYPE,
					 sizeof(LBV_MOTION_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_MOTION_HALT:public LBV_MOTION_CMD_MSG {
  public:
    LBV_MOTION_HALT():LBV_MOTION_CMD_MSG(LBV_MOTION_HALT_TYPE,
					 sizeof(LBV_MOTION_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_MOTION_ABORT:public LBV_MOTION_CMD_MSG {
  public:
    LBV_MOTION_ABORT():LBV_MOTION_CMD_MSG(LBV_MOTION_ABORT_TYPE,
					  sizeof(LBV_MOTION_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_MOTION_SET_AOUT:public LBV_MOTION_CMD_MSG {
  public:
    LBV_MOTION_SET_AOUT():LBV_MOTION_CMD_MSG(LBV_MOTION_SET_AOUT_TYPE,
					     sizeof(LBV_MOTION_SET_AOUT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    unsigned char index;	// which to set
    double start;		// value at start
    double end;			// value at end
    unsigned char now;		// wether command is imediate or synched with motion
};

class LBV_MOTION_SET_DOUT:public LBV_MOTION_CMD_MSG {
  public:
    LBV_MOTION_SET_DOUT():LBV_MOTION_CMD_MSG(LBV_MOTION_SET_DOUT_TYPE,
					     sizeof(LBV_MOTION_SET_DOUT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    unsigned char index;	// which to set
    unsigned char start;	// binary value at start
    unsigned char end;		// binary value at end
    unsigned char now;		// wether command is imediate or synched with motion
};

class LBV_MOTION_ADAPTIVE:public LBV_MOTION_CMD_MSG {
  public:
    LBV_MOTION_ADAPTIVE():LBV_MOTION_CMD_MSG(LBV_MOTION_ADAPTIVE_TYPE,
					     sizeof(LBV_MOTION_ADAPTIVE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    unsigned char status;		// status=0 stop; status=1 start.
};

// LBV_MOTION status base class
class LBV_MOTION_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_MOTION_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
	heartbeat = 0;
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    uint32_t heartbeat;
};


// LBV_SPINDLE status base class
class LBV_SPINDLE_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_SPINDLE_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_SPINDLE_STAT:public LBV_SPINDLE_STAT_MSG {
  public:
    LBV_SPINDLE_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double speed;		// spindle speed in RPMs
    double spindle_scale;	// spindle over-ride
    double css_maximum;
    double css_factor;  // CSS Status
    int direction;		// 0 stopped, 1 forward, -1 reverse
    int brake;			// 0 released, 1 engaged
    int increasing;		// 1 increasing, -1 decreasing, 0 neither
    int enabled;		// non-zero means enabled
    int orient_state;
    int orient_fault;
    bool spindle_override_enabled;
    bool homed;
};

class LBV_MOTION_STAT:public LBV_MOTION_STAT_MSG {
  public:
    LBV_MOTION_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // aggregate of motion-related status classes
    LBV_TRAJ_STAT traj;
    LBV_JOINT_STAT joint[LBVMOT_MAX_JOINTS];
    LBV_AXIS_STAT axis[LBVMOT_MAX_AXIS];
    LBV_SPINDLE_STAT spindle[LBVMOT_MAX_SPINDLES];

    int synch_di[LBVMOT_MAX_DIO];  // motion inputs queried by interp
    int synch_do[LBVMOT_MAX_DIO];  // motion outputs queried by interp
    double analog_input[LBVMOT_MAX_AIO]; //motion analog inputs queried by interp
    double analog_output[LBVMOT_MAX_AIO]; //motion analog outputs queried by interp
    int debug;			// copy of LBV_DEBUG global
    int on_soft_limit;
    int external_offsets_applied;
    LbvPose eoffset_pose;
    int numExtraJoints;
};

// declarations for LBV_TASK classes

// LBV_TASK command base class
class LBV_TASK_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_TASK_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_INIT:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_INIT():LBV_TASK_CMD_MSG(LBV_TASK_INIT_TYPE,
				     sizeof(LBV_TASK_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_HALT:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_HALT():LBV_TASK_CMD_MSG(LBV_TASK_HALT_TYPE,
				     sizeof(LBV_TASK_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_ABORT:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_ABORT():LBV_TASK_CMD_MSG(LBV_TASK_ABORT_TYPE,
				      sizeof(LBV_TASK_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_SET_MODE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_SET_MODE():LBV_TASK_CMD_MSG(LBV_TASK_SET_MODE_TYPE,
					 sizeof(LBV_TASK_SET_MODE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    enum LBV_TASK_MODE_ENUM mode;
};

class LBV_TASK_SET_STATE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_SET_STATE():LBV_TASK_CMD_MSG(LBV_TASK_SET_STATE_TYPE,
					  sizeof(LBV_TASK_SET_STATE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    enum LBV_TASK_STATE_ENUM state;
};

class LBV_TASK_PLAN_OPEN:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_OPEN():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_OPEN_TYPE,
					  sizeof(LBV_TASK_PLAN_OPEN)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    char file[LINELEN];
};

class LBV_TASK_PLAN_RUN:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_RUN():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_RUN_TYPE,
					 sizeof(LBV_TASK_PLAN_RUN)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int line;			// line to run from; 0 or 1 means from start,
    // negative means run through to verify
};

class LBV_TASK_PLAN_READ:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_READ():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_READ_TYPE,
					  sizeof(LBV_TASK_PLAN_READ)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_EXECUTE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_EXECUTE():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_EXECUTE_TYPE,
					     sizeof(LBV_TASK_PLAN_EXECUTE))
    {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    char command[LINELEN];
};

class LBV_TASK_PLAN_PAUSE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_PAUSE():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_PAUSE_TYPE,
					   sizeof(LBV_TASK_PLAN_PAUSE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_REVERSE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_REVERSE():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_REVERSE_TYPE,
					   sizeof(LBV_TASK_PLAN_REVERSE)) {
    };

};

class LBV_TASK_PLAN_FORWARD:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_FORWARD():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_FORWARD_TYPE,
					   sizeof(LBV_TASK_PLAN_FORWARD)) {
    };

};


class LBV_TASK_PLAN_STEP:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_STEP():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_STEP_TYPE,
					  sizeof(LBV_TASK_PLAN_STEP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_RESUME:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_RESUME():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_RESUME_TYPE,
					    sizeof(LBV_TASK_PLAN_RESUME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_END:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_END():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_END_TYPE,
					 sizeof(LBV_TASK_PLAN_END)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_CLOSE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_CLOSE():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_CLOSE_TYPE,
					   sizeof(LBV_TASK_PLAN_CLOSE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_INIT:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_INIT():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_INIT_TYPE,
					  sizeof(LBV_TASK_PLAN_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_SYNCH:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_SYNCH():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_SYNCH_TYPE,
					   sizeof(LBV_TASK_PLAN_SYNCH)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TASK_PLAN_SET_OPTIONAL_STOP:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_SET_OPTIONAL_STOP():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE,
					   sizeof(LBV_TASK_PLAN_SET_OPTIONAL_STOP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    
    bool state; //state == ON, optional stop is on (e.g. we stop on any stops)
};

class LBV_TASK_PLAN_SET_BLOCK_DELETE:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_SET_BLOCK_DELETE():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE,
					   sizeof(LBV_TASK_PLAN_SET_BLOCK_DELETE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    
    bool state; //state == ON, block delete is on, we ignore lines starting with "/"
};

class LBV_TASK_PLAN_OPTIONAL_STOP:public LBV_TASK_CMD_MSG {
  public:
    LBV_TASK_PLAN_OPTIONAL_STOP():LBV_TASK_CMD_MSG(LBV_TASK_PLAN_OPTIONAL_STOP_TYPE,
					   sizeof(LBV_TASK_PLAN_OPTIONAL_STOP)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    
};


// LBV_TASK status base class
class LBV_TASK_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_TASK_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
	heartbeat = 0;
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    uint32_t heartbeat;
};

class LBV_TASK_STAT:public LBV_TASK_STAT_MSG {
  public:
    LBV_TASK_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    enum LBV_TASK_MODE_ENUM mode;	// LBV_TASK_MODE_MANUAL, etc.
    enum LBV_TASK_STATE_ENUM state;	// LBV_TASK_STATE_ESTOP, etc.

    enum LBV_TASK_EXEC_ENUM execState;	// LBV_DONE,WAITING_FOR_MOTION, etc.
    enum LBV_TASK_INTERP_ENUM interpState;	// LBV_IDLE,READING,PAUSED,WAITING
    int callLevel;              // current subroutine level - 0 if not in a subroutine, > 0 otherwise
    int motionLine;		// line motion is executing-- may lag
    int currentLine;		// line currently executing
    int readLine;		// line interpreter has read to
    bool optional_stop_state;	// state of optional stop (== ON means we stop on M1)
    bool block_delete_state;	// state of block delete (== ON means we ignore lines starting with "/")
    bool input_timeout;		// has a timeout happened on digital input
    char file[LINELEN];
    char command[LINELEN];
    LbvPose g5x_offset;		// in user units, currently active
    int g5x_index;              // index of active g5x system
    LbvPose g92_offset;		// in user units, currently active
    double rotation_xy;
    LbvPose toolOffset;		// tool offset, in general pose form
    int activeGCodes[ACTIVE_G_CODES];
    int activeMCodes[ACTIVE_M_CODES];
    double activeSettings[ACTIVE_SETTINGS];
    CANON_UNITS programUnits;	// CANON_UNITS_INCHES, MM, CM

    int interpreter_errcode;	// return value from rs274ngc function 
    // (only useful for new interpreter.)
    int task_paused;		// non-zero means task is paused
    double delayLeft;           // delay time left of G4, M66..
    int queuedMDIcommands;      // current length of MDI input queue
};

// declarations for LBV_TOOL classes

// LBV_TOOL command base class
class LBV_TOOL_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_TOOL_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TOOL_INIT:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_INIT():LBV_TOOL_CMD_MSG(LBV_TOOL_INIT_TYPE,
				     sizeof(LBV_TOOL_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TOOL_HALT:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_HALT():LBV_TOOL_CMD_MSG(LBV_TOOL_HALT_TYPE,
				     sizeof(LBV_TOOL_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TOOL_ABORT:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_ABORT():LBV_TOOL_CMD_MSG(LBV_TOOL_ABORT_TYPE,
				      sizeof(LBV_TOOL_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int reason;		//  convey reason for abort to iocontrol
};

class LBV_TOOL_PREPARE:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_PREPARE():LBV_TOOL_CMD_MSG(LBV_TOOL_PREPARE_TYPE,
					sizeof(LBV_TOOL_PREPARE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int pocket;
    int tool;
};

class LBV_TOOL_LOAD:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_LOAD():LBV_TOOL_CMD_MSG(LBV_TOOL_LOAD_TYPE,
				     sizeof(LBV_TOOL_LOAD)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TOOL_UNLOAD:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_UNLOAD():LBV_TOOL_CMD_MSG(LBV_TOOL_UNLOAD_TYPE,
				       sizeof(LBV_TOOL_UNLOAD)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TOOL_LOAD_TOOL_TABLE:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_LOAD_TOOL_TABLE():LBV_TOOL_CMD_MSG
	(LBV_TOOL_LOAD_TOOL_TABLE_TYPE, sizeof(LBV_TOOL_LOAD_TOOL_TABLE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    char file[LINELEN];		// name of tool table, empty means default
};

class LBV_TOOL_SET_OFFSET:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_SET_OFFSET():LBV_TOOL_CMD_MSG(LBV_TOOL_SET_OFFSET_TYPE,
					   sizeof(LBV_TOOL_SET_OFFSET)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int pocket;
    int toolno;
    LbvPose offset;
    double diameter;
    double frontangle;
    double backangle;
    int    orientation;
};

class LBV_TOOL_SET_NUMBER:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_SET_NUMBER():LBV_TOOL_CMD_MSG(LBV_TOOL_SET_NUMBER_TYPE,
					   sizeof(LBV_TOOL_SET_NUMBER)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int tool; //number to use for currently loaded tool
};

class LBV_TOOL_START_CHANGE:public LBV_TOOL_CMD_MSG {
  public:
    LBV_TOOL_START_CHANGE():LBV_TOOL_CMD_MSG(LBV_TOOL_START_CHANGE_TYPE,
					     sizeof(LBV_TOOL_START_CHANGE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

// LBV_TOOL status base class
class LBV_TOOL_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_TOOL_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_TOOL_STAT:public LBV_TOOL_STAT_MSG {
  public:
    LBV_TOOL_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);
    LBV_TOOL_STAT operator =(LBV_TOOL_STAT s);	// need this for [] members

    int pocketPrepped;		// pocket ready for loading from
    int toolInSpindle;		// tool loaded, 0 is no tool
    CANON_TOOL_TABLE toolTable[CANON_POCKETS_MAX];
};

// LBV_AUX type declarations

// LBV_AUX command base class
class LBV_AUX_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_AUX_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_AUX_ESTOP_ON:public LBV_AUX_CMD_MSG {
  public:
    LBV_AUX_ESTOP_ON():LBV_AUX_CMD_MSG(LBV_AUX_ESTOP_ON_TYPE,
				       sizeof(LBV_AUX_ESTOP_ON)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_AUX_ESTOP_OFF:public LBV_AUX_CMD_MSG {
  public:
    LBV_AUX_ESTOP_OFF():LBV_AUX_CMD_MSG(LBV_AUX_ESTOP_OFF_TYPE,
					sizeof(LBV_AUX_ESTOP_OFF)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_AUX_ESTOP_RESET:public LBV_AUX_CMD_MSG {
  public:
    LBV_AUX_ESTOP_RESET():LBV_AUX_CMD_MSG(LBV_AUX_ESTOP_RESET_TYPE,
					sizeof(LBV_AUX_ESTOP_RESET)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_AUX_INPUT_WAIT:public LBV_AUX_CMD_MSG {
  public:
    LBV_AUX_INPUT_WAIT():LBV_AUX_CMD_MSG(LBV_AUX_INPUT_WAIT_TYPE,
					sizeof(LBV_AUX_INPUT_WAIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int index;			// input channel to wait for
    int input_type;		// DIGITAL or ANALOG
    int wait_type;		// 0 - immediate, 1- rise, 2 - fall, 3 - be high, 4 - be low
    double timeout;		// timeout for waiting
};


// LBV_AUX status base class
class LBV_AUX_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_AUX_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_AUX_STAT:public LBV_AUX_STAT_MSG {
  public:
    LBV_AUX_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int estop;			// non-zero means estopped
};

// LBV_SPINDLE type declarations

// LBV_SPINDLE command base class
class LBV_SPINDLE_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_SPINDLE_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
};

class LBV_SPINDLE_SPEED:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_SPEED():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_SPEED_TYPE,
					    sizeof(LBV_SPINDLE_SPEED)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
    double speed;   // commanded speed in RPMs or maximum speed for CSS
    double factor;  // Zero for constant RPM.  numerator of speed for CSS
    double xoffset; // X axis offset compared to center of rotation, for CSS
};

class LBV_SPINDLE_ORIENT:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_ORIENT():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_ORIENT_TYPE,
					    sizeof(LBV_SPINDLE_ORIENT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
    double orientation;   // desired spindle position
    int    mode;   
};

class LBV_SPINDLE_WAIT_ORIENT_COMPLETE:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_WAIT_ORIENT_COMPLETE():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE,
					    sizeof(LBV_SPINDLE_WAIT_ORIENT_COMPLETE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
    double timeout;   // how long to wait until spindle orient completes; > 0
};


class LBV_SPINDLE_ON:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_ON():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_ON_TYPE,
					 sizeof(LBV_SPINDLE_ON)),
	speed(0), factor(0), xoffset(0), wait_for_spindle_at_speed(1)  {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;    // the spindle to be turned on
    double speed;   // commanded speed in RPMs or maximum speed for CSS
    double factor;  // Zero for constant RPM.  numerator of speed for CSS
    double xoffset; // X axis offset compared to center of rotation, for CSS
    int wait_for_spindle_at_speed;
};

class LBV_SPINDLE_OFF:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_OFF():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_OFF_TYPE,
					  sizeof(LBV_SPINDLE_OFF)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;    // the spindle to be turned off
};

class LBV_SPINDLE_INCREASE:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_INCREASE():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_INCREASE_TYPE,
					       sizeof
					       (LBV_SPINDLE_INCREASE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int spindle;        // the spindle to be increased
    double speed;		// commanded speed in RPMs
};

class LBV_SPINDLE_DECREASE:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_DECREASE():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_DECREASE_TYPE,
					       sizeof
					       (LBV_SPINDLE_DECREASE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;        // the spindle to be decreased
    double speed;		// commanded speed in RPMs
};

class LBV_SPINDLE_CONSTANT:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_CONSTANT():LBV_SPINDLE_CMD_MSG(LBV_SPINDLE_CONSTANT_TYPE,
					       sizeof
					       (LBV_SPINDLE_CONSTANT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;        // the spindle to be constanted?
    double speed;		// commanded speed in RPMs
};

class LBV_SPINDLE_BRAKE_RELEASE:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_BRAKE_RELEASE():LBV_SPINDLE_CMD_MSG
	(LBV_SPINDLE_BRAKE_RELEASE_TYPE,
	 sizeof(LBV_SPINDLE_BRAKE_RELEASE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
};

class LBV_SPINDLE_BRAKE_ENGAGE:public LBV_SPINDLE_CMD_MSG {
  public:
    LBV_SPINDLE_BRAKE_ENGAGE():LBV_SPINDLE_CMD_MSG
	(LBV_SPINDLE_BRAKE_ENGAGE_TYPE, sizeof(LBV_SPINDLE_BRAKE_ENGAGE)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int spindle;
};

// LBV_COOLANT type declarations

// LBV_COOLANT command base class
class LBV_COOLANT_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_COOLANT_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_COOLANT_MIST_ON:public LBV_COOLANT_CMD_MSG {
  public:
    LBV_COOLANT_MIST_ON():LBV_COOLANT_CMD_MSG(LBV_COOLANT_MIST_ON_TYPE,
					      sizeof(LBV_COOLANT_MIST_ON))
    {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_COOLANT_MIST_OFF:public LBV_COOLANT_CMD_MSG {
  public:
    LBV_COOLANT_MIST_OFF():LBV_COOLANT_CMD_MSG(LBV_COOLANT_MIST_OFF_TYPE,
					       sizeof
					       (LBV_COOLANT_MIST_OFF)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_COOLANT_FLOOD_ON:public LBV_COOLANT_CMD_MSG {
  public:
    LBV_COOLANT_FLOOD_ON():LBV_COOLANT_CMD_MSG(LBV_COOLANT_FLOOD_ON_TYPE,
					       sizeof
					       (LBV_COOLANT_FLOOD_ON)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_COOLANT_FLOOD_OFF:public LBV_COOLANT_CMD_MSG {
  public:
    LBV_COOLANT_FLOOD_OFF():LBV_COOLANT_CMD_MSG(LBV_COOLANT_FLOOD_OFF_TYPE,
						sizeof
						(LBV_COOLANT_FLOOD_OFF)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

// LBV_COOLANT status base class
class LBV_COOLANT_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_COOLANT_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_COOLANT_STAT:public LBV_COOLANT_STAT_MSG {
  public:
    LBV_COOLANT_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int mist;			// 0 off, 1 on
    int flood;			// 0 off, 1 on
};

// LBV_LUBE type declarations

// LBV_LUBE command base class
class LBV_LUBE_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_LUBE_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_LUBE_ON:public LBV_LUBE_CMD_MSG {
  public:
    LBV_LUBE_ON():LBV_LUBE_CMD_MSG(LBV_LUBE_ON_TYPE, sizeof(LBV_LUBE_ON)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_LUBE_OFF:public LBV_LUBE_CMD_MSG {
  public:
    LBV_LUBE_OFF():LBV_LUBE_CMD_MSG(LBV_LUBE_OFF_TYPE,
				    sizeof(LBV_LUBE_OFF)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

// LBV_LUBE status base class
class LBV_LUBE_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_LUBE_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_LUBE_STAT:public LBV_LUBE_STAT_MSG {
  public:
    LBV_LUBE_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    int on;			// 0 off, 1 on
    int level;			// 0 low, 1 okay
};

// LBV_IO is aggregate of all LBV IO-related status classes

// LBV_IO command base class
class LBV_IO_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_IO_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_IO_INIT:public LBV_IO_CMD_MSG {
  public:
    LBV_IO_INIT():LBV_IO_CMD_MSG(LBV_IO_INIT_TYPE, sizeof(LBV_IO_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_IO_HALT:public LBV_IO_CMD_MSG {
  public:
    LBV_IO_HALT():LBV_IO_CMD_MSG(LBV_IO_HALT_TYPE, sizeof(LBV_IO_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_IO_ABORT:public LBV_IO_CMD_MSG {
  public:
    LBV_IO_ABORT():LBV_IO_CMD_MSG(LBV_IO_ABORT_TYPE, sizeof(LBV_IO_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_IO_SET_CYCLE_TIME:public LBV_IO_CMD_MSG {
  public:
    LBV_IO_SET_CYCLE_TIME():LBV_IO_CMD_MSG(LBV_IO_SET_CYCLE_TIME_TYPE,
					   sizeof(LBV_IO_SET_CYCLE_TIME)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    double cycleTime;
};

// LBV_IO status base class
class LBV_IO_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_IO_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
	heartbeat = 0;
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    uint32_t heartbeat;
};

class LBV_IO_STAT:public LBV_IO_STAT_MSG {
  public:
    LBV_IO_STAT():LBV_IO_STAT_MSG(LBV_IO_STAT_TYPE, sizeof(LBV_IO_STAT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // top-level stuff
    double cycleTime;
    int debug;			// copy of LBV_DEBUG global
    int reason;			// to communicate abort/fault cause
    int fault;                  //  0 on succes, 1 on fault during M6
    // aggregate of IO-related status classes
    LBV_TOOL_STAT tool;
    LBV_COOLANT_STAT coolant;
    LBV_AUX_STAT aux;
    LBV_LUBE_STAT lube;

};

// LBV is aggregate of LBV_TASK, LBV_TRAJ, LBV_IO, etc.

// LBV command base class
class LBV_CMD_MSG:public RCS_CMD_MSG {
  public:
    LBV_CMD_MSG(NMLTYPE t, size_t s):RCS_CMD_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_INIT:public LBV_CMD_MSG {
  public:
    LBV_INIT():LBV_CMD_MSG(LBV_INIT_TYPE, sizeof(LBV_INIT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_HALT:public LBV_CMD_MSG {
  public:
    LBV_HALT():LBV_CMD_MSG(LBV_HALT_TYPE, sizeof(LBV_HALT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_ABORT:public LBV_CMD_MSG {
  public:
    LBV_ABORT():LBV_CMD_MSG(LBV_ABORT_TYPE, sizeof(LBV_ABORT)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

/** queue a call to a task-time Python plugin method
 * call is expected to be a tuple of (method,pickled posargs,pickled kwargs)
 */
class LBV_EXEC_PLUGIN_CALL:public LBV_CMD_MSG {
  public:
    LBV_EXEC_PLUGIN_CALL():LBV_CMD_MSG(LBV_EXEC_PLUGIN_CALL_TYPE,
				    sizeof(LBV_EXEC_PLUGIN_CALL)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int len;
    char call[900]; // MAX_NML_COMMAND_SIZE-100;
};

/** queue a call to a task-time Io Task Python plugin method
 * call is expected to be a tuple of (method,pickled posargs,pickled kwargs)
 */
class LBV_IO_PLUGIN_CALL:public LBV_CMD_MSG {
  public:
    LBV_IO_PLUGIN_CALL():LBV_CMD_MSG(LBV_IO_PLUGIN_CALL_TYPE,
				    sizeof(LBV_IO_PLUGIN_CALL)) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
    int len;
    char call[900]; // MAX_NML_COMMAND_SIZE-100;
};


// LBV status base class

class LBV_STAT_MSG:public RCS_STAT_MSG {
  public:
    LBV_STAT_MSG(NMLTYPE t, size_t s):RCS_STAT_MSG(t, s) {
    };

    // For internal NML/CMS use only.
    void update(CMS * cms);
};

class LBV_STAT:public LBV_STAT_MSG {
  public:
    LBV_STAT();

    // For internal NML/CMS use only.
    void update(CMS * cms);

    // the top-level LBV_TASK status class
    LBV_TASK_STAT task;

    // subordinate status classes
    LBV_MOTION_STAT motion;
    LBV_IO_STAT io;

    int debug;			// copy of LBV_DEBUG global
};

/*
   Declarations of LBV status class implementations, for major subsystems.
   These are defined in the appropriate main() files, and referenced
   by code in other files to get LBV status.
   */


#endif
