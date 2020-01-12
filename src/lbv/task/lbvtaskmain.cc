/********************************************************************
* Description: lbvtaskmain.cc
*   Main program for LBV task level
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
/*
  Principles of operation:

  1.  The main program calls lbvTaskPlan() and lbvTaskExecute() cyclically.

  2.  lbvTaskPlan() reads the new command, and decides what to do with
  it based on the mode (manual, auto, mdi) or state (estop, on) of the
  machine. Many of the commands just go out immediately to the
  subsystems (motion and IO). In auto mode, the interpreter is called
  and as a result the interp_list is appended with NML commands.

  3.  lbvTaskExecute() executes a big switch on execState. If it's done,
  it gets the next item off the interp_list, and sets execState to the
  preconditions for that. These preconditions include waiting for motion,
  waiting for IO, etc. Once they are satisfied, it issues the command, and
  sets execState to the postconditions. Once those are satisfied, it gets
  the next item off the interp_list, and so on.

  4.  preconditions and postconditions are only looked at in conjunction
  with commands on the interp_list. Immediate commands won't have any
  pre- or postconditions associated with them looked at.

  5.  At this point, nothing in this file adds anything to the interp_list.
  This could change, for example, when defining pre- and postconditions for
  jog or home commands. If this is done, make sure that the corresponding
  abort command clears out the interp_list.

  6. Single-stepping is handled in checkPreconditions() as the first
  condition. If we're in single-stepping mode, as indicated by the
  variable 'stepping', we set the state to waiting-for-step. This
  polls on the variable 'steppingWait' which is reset to zero when a
  step command is received, and set to one when the command is
  issued.
  */

#include <stdio.h>		// vsprintf()
#include <string.h>		// strcpy()
#include <stdarg.h>		// va_start()
#include <stdlib.h>		// exit()
#include <signal.h>		// signal(), SIGINT
#include <float.h>		// DBL_MAX
#include <sys/types.h>		// pid_t
#include <unistd.h>		// fork()
#include <sys/wait.h>		// waitpid(), WNOHANG, WIFEXITED
#include <ctype.h>		// isspace()
#include <libintl.h>
#include <locale.h>
#include "usrmotintf.h"

#if 0
// Enable this to niftily trap floating point exceptions for debugging
#include <fpu_control.h>
fpu_control_t __fpu_control = _FPU_IEEE & ~(_FPU_MASK_IM | _FPU_MASK_ZM | _FPU_MASK_OM);
#endif

#include "rcs.hh"		// NML classes, nmlErrorFormat()
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"
#include "canon.hh"		// CANON_TOOL_TABLE stuff
#include "inifile.hh"		// INIFILE
#include "interpl.hh"		// NML_INTERP_LIST, interp_list
#include "lbvglb.h"		// LBV_INIFILE,NMLFILE, LBV_TASK_CYCLE_TIME
#include "interp_return.hh"	// public interpreter return values
#include "interp_internal.hh"	// interpreter private definitions
#include "rcs_print.hh"
#include "timer.hh"
#include "nml_oi.hh"
#include "task.hh"		// lbvTaskCommand etc
#include "taskclass.hh"
#include "motion.h"             // LBVMOT_ORIENT_*
#include "inihal.hh"

static lbvmot_config_t lbvmotConfig;

/* time after which the user interface is declared dead
 * because it would'nt read any more messages
 */
#define DEFAULT_LBV_UI_TIMEOUT 5.0


// NML channels
static RCS_CMD_CHANNEL *lbvCommandBuffer = 0;
static RCS_STAT_CHANNEL *lbvStatusBuffer = 0;
static NML *lbvErrorBuffer = 0;

// NML command channel data pointer
static RCS_CMD_MSG *lbvCommand = 0;

// global LBV status
LBV_STAT *lbvStatus = 0;

// timer stuff
static RCS_TIMER *timer = 0;

// flag signifying that ini file [TASK] CYCLE_TIME is <= 0.0, so
// we should not delay at all between cycles. This means also that
// the LBV_TASK_CYCLE_TIME global will be set to the measured cycle
// time each cycle, in case other code references this.
static int lbvTaskNoDelay = 0;
// flag signifying that on the next loop, there should be no delay.
// this is set when transferring trajectory data from userspace to kernel
// space, annd reset otherwise.
static int lbvTaskEager = 0;

static int no_force_homing = 0; // forces the user to home first before allowing MDI and Program run
//can be overriden by [TRAJ]NO_FORCE_HOMING=1

static double LBV_TASK_CYCLE_TIME_ORIG = 0.0;

// delay counter
static double taskExecDelayTimeout = 0.0;

// lbvTaskIssueCommand issues command immediately
static int lbvTaskIssueCommand(NMLmsg * cmd);

// pending command to be sent out by lbvTaskExecute()
NMLmsg *lbvTaskCommand = 0;

// signal handling code to stop main loop
int done;
static int lbvtask_shutdown(void);
extern void backtrace(int signo);
int _task = 1; // control preview behaviour when remapping

// for operator display on iocontrol signalling a toolchanger fault if io.fault is set
// %d receives io.reason
static const char *io_error = "toolchanger error %d";

extern void setup_signal_handlers(); // backtrace, gdb-in-new-window supportx

int all_homed(void) {
    for (int i = 0; i < lbvStatus->motion.traj.joints; i++) {
        if(!lbvStatus->motion.joint[i].homed) // XXX
            return 0;
    }
    return 1;
}

void lbvtask_quit(int sig)
{
    // set main's done flag
    done = 1;
    // restore signal handler
    signal(sig, lbvtask_quit);
}

/* make sure at least space bytes are available on
 * error channel; wait a bit to drain if needed
 */
int lbvErrorBufferOKtoWrite(int space, const char *caller)
{
    // check channel for validity
    if (lbvErrorBuffer == NULL)
	return -1;
    if (!lbvErrorBuffer->valid())
	return -1;

    double send_errorchan_timout = etime() + DEFAULT_LBV_UI_TIMEOUT;

    while (etime() < send_errorchan_timout) {
	if (lbvErrorBuffer->get_space_available() < space) {
	    esleep(0.01);
	    continue;
	} else {
	    break;
	}
    }
    if (etime() >= send_errorchan_timout) {
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print("timeout waiting for error channel to drain, caller=`%s' request=%d\n", caller,space);
	}
	return -1;
    } else {
	// printf("--- %d bytes available after %f seconds\n", space, etime() - send_errorchan_timout + DEFAULT_LBV_UI_TIMEOUT);
    }
    return 0;
}


// implementation of LBV error logger
int lbvOperatorError(int id, const char *fmt, ...)
{
    LBV_OPERATOR_ERROR error_msg;
    va_list ap;

    if ( lbvErrorBufferOKtoWrite(sizeof(error_msg) * 2, "lbvOperatorError"))
	return -1;

    if (NULL == fmt) {
	return -1;
    }
    if (0 == *fmt) {
	return -1;
    }
    // prepend error code, leave off 0 ad-hoc code
    error_msg.error[0] = 0;
    if (0 != id) {
	snprintf(error_msg.error, sizeof(error_msg.error), "[%d] ", id);
    }
    // append error string
    va_start(ap, fmt);
    vsnprintf(&error_msg.error[strlen(error_msg.error)], 
	      sizeof(error_msg.error) - strlen(error_msg.error), fmt, ap);
    va_end(ap);

    // force a NULL at the end for safety
    error_msg.error[LINELEN - 1] = 0;

    // write it
    rcs_print("%s\n", error_msg.error);
    return lbvErrorBuffer->write(error_msg);
}

int lbvOperatorText(int id, const char *fmt, ...)
{
    LBV_OPERATOR_TEXT text_msg;
    va_list ap;

    if ( lbvErrorBufferOKtoWrite(sizeof(text_msg) * 2, "lbvOperatorText"))
	return -1;

    // write args to NML message (ignore int text code)
    va_start(ap, fmt);
    vsnprintf(text_msg.text, sizeof(text_msg.text), fmt, ap);
    va_end(ap);

    // force a NULL at the end for safety
    text_msg.text[LINELEN - 1] = 0;

    // write it
    return lbvErrorBuffer->write(text_msg);
}

int lbvOperatorDisplay(int id, const char *fmt, ...)
{
    LBV_OPERATOR_DISPLAY display_msg;
    va_list ap;

    if ( lbvErrorBufferOKtoWrite(sizeof(display_msg) * 2, "lbvOperatorDisplay"))
	return -1;

    // write args to NML message (ignore int display code)
    va_start(ap, fmt);
    vsnprintf(display_msg.display, sizeof(display_msg.display), fmt, ap);
    va_end(ap);

    // force a NULL at the end for safety
    display_msg.display[LINELEN - 1] = 0;

    // write it
    return lbvErrorBuffer->write(display_msg);
}

/*
  handling of LBV_SYSTEM_CMD
 */

/* convert string to arg/argv set */

static int argvize(const char *src, char *dst, char *argv[], int len)
{
    char *bufptr;
    int argvix;
    char inquote;
    char looking;

    strncpy(dst, src, len);
    dst[len - 1] = 0;
    bufptr = dst;
    inquote = 0;
    argvix = 0;
    looking = 1;

    while (0 != *bufptr) {
	if (*bufptr == '"') {
	    *bufptr = 0;
	    if (inquote) {
		inquote = 0;
		looking = 1;
	    } else {
		inquote = 1;
	    }
	} else if (isspace(*bufptr) && !inquote) {
	    looking = 1;
	    *bufptr = 0;
	} else if (looking) {
	    looking = 0;
	    argv[argvix] = bufptr;
	    argvix++;
	}
	bufptr++;
    }

    argv[argvix] = 0;		// null-terminate the argv list

    return argvix;
}

static pid_t lbvSystemCmdPid = 0;

int lbvSystemCmd(char *s)
{
    char buffer[LBV_SYSTEM_CMD_LEN];
    char *argv[LBV_SYSTEM_CMD_LEN / 2 + 1];

    if (0 != lbvSystemCmdPid) {
	// something's already running, and we can only handle one
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print
		("lbvSystemCmd: abandoning process %d, running ``%s''\n",
		 lbvSystemCmdPid, s);
	}
    }

    lbvSystemCmdPid = fork();

    if (-1 == lbvSystemCmdPid) {
	// we're still the parent, with no child created
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print("system command ``%s'' can't be executed\n", s);
	}
	return -1;
    }

    if (0 == lbvSystemCmdPid) {
	// we're the child
	// convert string to argc/argv
	argvize(s, buffer, argv, LBV_SYSTEM_CMD_LEN);
	execvp(argv[0], argv);
	// if we get here, we didn't exec
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print("lbvSystemCmd: can't execute ``%s''\n", s);
	}
	exit(-1);
    }
    // else we're the parent
    return 0;
}

// shorthand typecasting ptrs
static LBV_JOINT_HALT *joint_halt_msg;
static LBV_JOINT_DISABLE *disable_msg;
static LBV_JOINT_ENABLE *enable_msg;
static LBV_JOINT_HOME *home_msg;
static LBV_JOINT_UNHOME *unhome_msg;
static LBV_JOG_CONT *jog_cont_msg;
static LBV_JOG_STOP *jog_stop_msg;
static LBV_JOG_INCR *jog_incr_msg;
static LBV_JOG_ABS *jog_abs_msg;
static LBV_JOINT_SET_BACKLASH *set_backlash_msg;
static LBV_JOINT_SET_HOMING_PARAMS *set_homing_params_msg;
static LBV_JOINT_SET_FERROR *set_ferror_msg;
static LBV_JOINT_SET_MIN_FERROR *set_min_ferror_msg;
static LBV_JOINT_SET_MAX_POSITION_LIMIT *set_max_limit_msg;
static LBV_JOINT_SET_MIN_POSITION_LIMIT *set_min_limit_msg;
static LBV_JOINT_OVERRIDE_LIMITS *joint_lim_msg;
//static LBV_JOINT_SET_OUTPUT *axis_output_msg;
static LBV_JOINT_LOAD_COMP *joint_load_comp_msg;
//static LBV_AXIS_SET_STEP_PARAMS *set_step_params_msg;

static LBV_TRAJ_SET_SCALE *lbvTrajSetScaleMsg;
static LBV_TRAJ_SET_RAPID_SCALE *lbvTrajSetRapidScaleMsg;
static LBV_TRAJ_SET_MAX_VELOCITY *lbvTrajSetMaxVelocityMsg;
static LBV_TRAJ_SET_SPINDLE_SCALE *lbvTrajSetSpindleScaleMsg;
static LBV_TRAJ_SET_VELOCITY *lbvTrajSetVelocityMsg;
static LBV_TRAJ_SET_ACCELERATION *lbvTrajSetAccelerationMsg;
static LBV_TRAJ_LINEAR_MOVE *lbvTrajLinearMoveMsg;
static LBV_TRAJ_CIRCULAR_MOVE *lbvTrajCircularMoveMsg;
static LBV_TRAJ_DELAY *lbvTrajDelayMsg;
static LBV_TRAJ_SET_TERM_COND *lbvTrajSetTermCondMsg;
static LBV_TRAJ_SET_SPINDLESYNC *lbvTrajSetSpindlesyncMsg;

// These classes are commented out because the compiler
// complains that they are "defined but not used".
//static LBV_MOTION_SET_AOUT *lbvMotionSetAoutMsg;
//static LBV_MOTION_SET_DOUT *lbvMotionSetDoutMsg;

static LBV_SPINDLE_SPEED *spindle_speed_msg;
static LBV_SPINDLE_ORIENT *spindle_orient_msg;
static LBV_SPINDLE_WAIT_ORIENT_COMPLETE *wait_spindle_orient_complete_msg;
static LBV_SPINDLE_ON *spindle_on_msg;
static LBV_SPINDLE_OFF *spindle_off_msg;
static LBV_SPINDLE_BRAKE_ENGAGE *spindle_brake_engage_msg;
static LBV_SPINDLE_BRAKE_RELEASE *spindle_brake_release_msg;
static LBV_SPINDLE_INCREASE *spindle_increase_msg;
static LBV_SPINDLE_DECREASE *spindle_decrease_msg;
static LBV_SPINDLE_CONSTANT *spindle_constant_msg;
static LBV_TOOL_PREPARE *tool_prepare_msg;
static LBV_TOOL_LOAD_TOOL_TABLE *load_tool_table_msg;
static LBV_TOOL_SET_OFFSET *lbv_tool_set_offset_msg;
static LBV_TOOL_SET_NUMBER *lbv_tool_set_number_msg;
static LBV_TASK_SET_MODE *mode_msg;
static LBV_TASK_SET_STATE *state_msg;
static LBV_TASK_PLAN_RUN *run_msg;
static LBV_TASK_PLAN_EXECUTE *execute_msg;
static LBV_TASK_PLAN_OPEN *open_msg;
static LBV_TASK_PLAN_SET_OPTIONAL_STOP *os_msg;
static LBV_TASK_PLAN_SET_BLOCK_DELETE *bd_msg;

static LBV_AUX_INPUT_WAIT *lbvAuxInputWaitMsg;
static int lbvAuxInputWaitType = 0;
static int lbvAuxInputWaitIndex = -1;

// commands we compose here
static LBV_TASK_PLAN_RUN taskPlanRunCmd;	// 16-Aug-1999 FMP
static LBV_TASK_PLAN_INIT taskPlanInitCmd;
static LBV_TASK_PLAN_SYNCH taskPlanSynchCmd;

static int interpResumeState = LBV_TASK_INTERP_IDLE;
static int programStartLine = 0;	// which line to run program from
// how long the interp list can be

int stepping = 0;
int steppingWait = 0;
static int steppedLine = 0;

// Variables to handle MDI call interrupts
// Depth of call level before interrupted MDI call
static int mdi_execute_level = -1;
// Schedule execute(0) command
static int mdi_execute_next = 0;
// Wait after interrupted command
static int mdi_execute_wait = 0;
// Side queue to store MDI commands
static NML_INTERP_LIST mdi_execute_queue;

// MDI input queue
static NML_INTERP_LIST mdi_input_queue;
#define  MAX_MDI_QUEUE 10
static int max_mdi_queued_commands = MAX_MDI_QUEUE;

/*
  checkInterpList(NML_INTERP_LIST *il, LBV_STAT *stat) takes a pointer
  to an interpreter list and a pointer to the LBV status, pops each NML
  message off the list, and checks it against limits, resource availability,
  etc. in the status.

  It returns 0 if all messages check out, -1 if any of them fail. If one
  fails, the rest of the list is not checked.
 */
static int checkInterpList(NML_INTERP_LIST * il, LBV_STAT * stat)
{
    NMLmsg *cmd = 0;
    // let's create some shortcuts to casts at compile time
#define operator_error_msg ((LBV_OPERATOR_ERROR *) cmd)
#define linear_move ((LBV_TRAJ_LINEAR_MOVE *) cmd)
#define circular_move ((LBV_TRAJ_CIRCULAR_MOVE *) cmd)

    while (il->len() > 0) {
	cmd = il->get();

	switch (cmd->type) {

	case LBV_OPERATOR_ERROR_TYPE:
	    lbvOperatorError(operator_error_msg->id, "%s",
			     operator_error_msg->error);
	    break;

//FIXME: there was limit checking tests below, see if they were needed
	case LBV_TRAJ_LINEAR_MOVE_TYPE:
	    break;

	case LBV_TRAJ_CIRCULAR_MOVE_TYPE:
	    break;

	default:
	    break;
	}
    }

    return 0;

    // get rid of the compile-time cast shortcuts
#undef circular_move_msg
#undef linear_move_msg
#undef operator_error_msg
}
extern int lbvTaskMopup();

void readahead_reading(void)
{
    int readRetval;
    int execRetval;

		if (interp_list.len() <= lbv_task_interp_max_len) {
                    int count = 0;
interpret_again:
		    if (lbvTaskPlanIsWait()) {
			// delay reading of next line until all is done
			if (interp_list.len() == 0 &&
			    lbvTaskCommand == 0 &&
			    lbvStatus->task.execState ==
			    LBV_TASK_EXEC_DONE) {
			    lbvTaskPlanClearWait();
			 }
		    } else {
			readRetval = lbvTaskPlanRead();
			/*! \todo MGS FIXME
			   This if() actually evaluates to if (readRetval != INTERP_OK)...
			   *** Need to look at all calls to things that return INTERP_xxx values! ***
			   MGS */
			if (readRetval > INTERP_MIN_ERROR
				|| readRetval == INTERP_ENDFILE
				|| readRetval == INTERP_EXIT
				|| readRetval == INTERP_EXECUTE_FINISH) {
			    /* lbvTaskPlanRead retval != INTERP_OK
			       Signal to the rest of the system that that the interp
			       is now in a paused state. */
			    /*! \todo FIXME The above test *should* be reduced to:
			       readRetVal != INTERP_OK
			       (N.B. Watch for negative error codes.) */
			    lbvStatus->task.interpState =
				LBV_TASK_INTERP_WAITING;
			} else {
			    // got a good line
			    // record the line number and command
			    lbvStatus->task.readLine = lbvTaskPlanLine();

			    lbvTaskPlanCommand((char *) &lbvStatus->task.
					       command);
			    // and execute it
			    execRetval = lbvTaskPlanExecute(0);
			    // line number may need update after
			    // returns from subprograms in external
			    // files
			    lbvStatus->task.readLine = lbvTaskPlanLine();
			    if (execRetval > INTERP_MIN_ERROR) {
				lbvStatus->task.interpState =
				    LBV_TASK_INTERP_WAITING;
				interp_list.clear();
				lbvAbortCleanup(LBV_ABORT_INTERPRETER_ERROR,
						"interpreter error"); 
			    } else if (execRetval == -1
				    || execRetval == INTERP_EXIT ) {
				lbvStatus->task.interpState =
				    LBV_TASK_INTERP_WAITING;
			    } else if (execRetval == INTERP_EXECUTE_FINISH) {
				// INTERP_EXECUTE_FINISH signifies
				// that no more reading should be done until
				// everything
				// outstanding is completed
				lbvTaskPlanSetWait();
				// and resynch interp WM
				lbvTaskQueueCommand(&taskPlanSynchCmd);
			    } else if (execRetval != 0) {
				// end of file
				lbvStatus->task.interpState =
				    LBV_TASK_INTERP_WAITING;
                                lbvStatus->task.motionLine = 0;
                                lbvStatus->task.readLine = 0;
			    } else {

				// executed a good line
			    }

			    // throw the results away if we're supposed to
			    // read
			    // through it
			    if ( programStartLine != 0 &&
				 lbvTaskPlanLevel() == 0 &&
				 ( programStartLine < 0 ||
				   lbvTaskPlanLine() <= programStartLine )) {
				// we're stepping over lines, so check them
				// for
				// limits, etc. and clear then out
				if (0 != checkInterpList(&interp_list,
							 lbvStatus)) {
				    // problem with actions, so do same as we
				    // did
				    // for a bad read from lbvTaskPlanRead()
				    // above
				    lbvStatus->task.interpState =
					LBV_TASK_INTERP_WAITING;
				}
				// and clear it regardless
				interp_list.clear();
			    }

			    if (lbvStatus->task.readLine < programStartLine &&
				lbvTaskPlanLevel() == 0) {
			    
				//update the position with our current position, as the other positions are only skipped through
				CANON_UPDATE_END_POINT(lbvStatus->motion.traj.actualPosition.tran.x,
						       lbvStatus->motion.traj.actualPosition.tran.y,
						       lbvStatus->motion.traj.actualPosition.tran.z,
						       lbvStatus->motion.traj.actualPosition.a,
						       lbvStatus->motion.traj.actualPosition.b,
						       lbvStatus->motion.traj.actualPosition.c,
						       lbvStatus->motion.traj.actualPosition.u,
						       lbvStatus->motion.traj.actualPosition.v,
						       lbvStatus->motion.traj.actualPosition.w);

				if ((lbvStatus->task.readLine + 1 == programStartLine)  &&
				    (lbvTaskPlanLevel() == 0))  {

				    lbvTaskPlanSynch();

                                    // reset programStartLine so we don't fall into our stepping routines
                                    // if we happen to execute lines before the current point later (due to subroutines).
                                    programStartLine = 0;
                                }
			    }

                            if (count++ < lbv_task_interp_max_len
                                    && lbvStatus->task.interpState == LBV_TASK_INTERP_READING
                                    && interp_list.len() <= lbv_task_interp_max_len * 2/3) {
                                goto interpret_again;
                            }

			}	// else read was OK, so execute
		    }		// else not lbvTaskPlanIsWait
		}		// if interp len is less than max
}

static void mdi_execute_abort(void)
{
    int queued_mdi_commands;

    // XXX: Reset needed?
    if (mdi_execute_wait || mdi_execute_next)
        lbvTaskPlanReset();
    mdi_execute_level = -1;
    mdi_execute_wait = 0;
    mdi_execute_next = 0;

    queued_mdi_commands = mdi_execute_queue.len();
    if (queued_mdi_commands > 0) {
        rcs_print("mdi_execute_abort: dropping %d queued MDI commands\n", queued_mdi_commands);
    }
    mdi_execute_queue.clear();
    lbvStatus->task.queuedMDIcommands = 0;

    lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
}

static void mdi_execute_hook(void)
{
    if (mdi_execute_wait && lbvTaskPlanIsWait()) {
	// delay reading of next line until all is done
	if (interp_list.len() == 0 &&
	    lbvTaskCommand == 0 &&
	    lbvStatus->task.execState ==
	    LBV_TASK_EXEC_DONE) {
	    lbvTaskPlanClearWait(); 
	    mdi_execute_wait = 0;
	    mdi_execute_hook();
	}
	return;
    }

    if (
        (mdi_execute_level < 0)
        && (mdi_execute_wait == 0)
        && (mdi_execute_queue.len() > 0)
        && (interp_list.len() == 0)
        && (lbvTaskCommand == NULL)
    ) {
	interp_list.append(mdi_execute_queue.get());
        lbvStatus->task.queuedMDIcommands = mdi_execute_queue.len();
	return;
    }

    // determine when a MDI command actually finishes normally.
    if (interp_list.len() == 0 &&
	lbvTaskCommand == 0 &&
	lbvStatus->task.execState ==  LBV_TASK_EXEC_DONE && 
	lbvStatus->task.interpState != LBV_TASK_INTERP_IDLE && 
	lbvStatus->motion.traj.queue == 0 &&
	lbvStatus->io.status == RCS_DONE && 
	!mdi_execute_wait && 
	!mdi_execute_next) {

	// finished. Check for dequeuing of queued MDI command is done in lbvTaskPlan().
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE)
	    rcs_print("mdi_execute_hook: MDI command '%s' done (remaining: %d)\n",
		      lbvStatus->task.command, mdi_input_queue.len());
	lbvStatus->task.command[0] = 0;
	lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
    }

    if (!mdi_execute_next) return;

    if (interp_list.len() > lbv_task_interp_max_len) return;

    mdi_execute_next = 0;

    LBV_TASK_PLAN_EXECUTE msg;
    msg.command[0] = (char) 0xff;

    interp_list.append(msg);
}

void readahead_waiting(void)
{
	// now handle call logic
	// check for subsystems done
	if (interp_list.len() == 0 &&
	    lbvTaskCommand == 0 &&
	    lbvStatus->motion.traj.queue == 0 &&
	    lbvStatus->io.status == RCS_DONE)
	    // finished
	{
	    int was_open = taskplanopen;
	    if (was_open) {
		lbvTaskPlanClose();
		if (lbv_debug & LBV_DEBUG_INTERP && was_open) {
		    rcs_print
			("lbvTaskPlanClose() called at %s:%d\n",
			 __FILE__, __LINE__);
		}
		// then resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
	    } else {
		lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
	    }
	    lbvStatus->task.readLine = 0;
	} else {
	    // still executing
        }
}

static bool allow_while_idle_type() {
    // allow for LBV_TASK_MODE_AUTO, LBV_TASK_MODE_MDI
    // expect immediate command
    RCS_CMD_MSG *lbvCommand;
    lbvCommand = lbvCommandBuffer->get_address();
    switch(lbvCommand->type) {
      case LBV_JOG_CONT_TYPE:
      case LBV_JOG_INCR_TYPE:
      case LBV_JOG_STOP_TYPE:
      case LBV_JOG_ABS_TYPE:
      case LBV_TRAJ_SET_TELEOP_ENABLE_TYPE:
       return 1;
       break;
    }
    return 0;
}

/*
  lbvTaskPlan()

  Planner for NC code or manual mode operations
  */
static int lbvTaskPlan(void)
{
    NMLTYPE type;
    int retval = 0;

    // check for new command
    if (lbvCommand->serial_number != lbvStatus->echo_serial_number) {
	// flag it here locally as a new command
	type = lbvCommand->type;
    } else {
	// no new command-- reset local flag
	type = 0;
    }

    // handle any new command
    switch (lbvStatus->task.state) {
    case LBV_TASK_STATE_OFF:
    case LBV_TASK_STATE_ESTOP:
    case LBV_TASK_STATE_ESTOP_RESET:

	// now switch on the mode
	switch (lbvStatus->task.mode) {
	case LBV_TASK_MODE_MANUAL:
	case LBV_TASK_MODE_AUTO:
	case LBV_TASK_MODE_MDI:

	    // now switch on the command
	    switch (type) {
	    case 0:
	    case LBV_NULL_TYPE:
		// no command
		break;

		// immediate commands
	    case LBV_JOINT_SET_BACKLASH_TYPE:
	    case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
	    case LBV_JOINT_DISABLE_TYPE:
	    case LBV_JOINT_ENABLE_TYPE:
	    case LBV_JOINT_SET_FERROR_TYPE:
	    case LBV_JOINT_SET_MIN_FERROR_TYPE:
	    case LBV_JOINT_ABORT_TYPE:
	    case LBV_JOINT_LOAD_COMP_TYPE:
	    case LBV_JOINT_UNHOME_TYPE:
	    case LBV_TRAJ_SET_SCALE_TYPE:
	    case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
	    case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
	    case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
	    case LBV_TRAJ_SET_FO_ENABLE_TYPE:
	    case LBV_TRAJ_SET_FH_ENABLE_TYPE:
	    case LBV_TRAJ_SET_SO_ENABLE_TYPE:
	    case LBV_TRAJ_SET_VELOCITY_TYPE:
	    case LBV_TRAJ_SET_ACCELERATION_TYPE:
	    case LBV_TASK_INIT_TYPE:
	    case LBV_TASK_SET_MODE_TYPE:
	    case LBV_TASK_SET_STATE_TYPE:
	    case LBV_TASK_PLAN_INIT_TYPE:
	    case LBV_TASK_PLAN_OPEN_TYPE:
	    case LBV_TASK_PLAN_CLOSE_TYPE:
	    case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	    case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	    case LBV_TASK_ABORT_TYPE:
	    case LBV_TASK_PLAN_SYNCH_TYPE:
	    case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	    case LBV_TRAJ_PROBE_TYPE:
	    case LBV_AUX_INPUT_WAIT_TYPE:
	    case LBV_MOTION_SET_DOUT_TYPE:
	    case LBV_MOTION_ADAPTIVE_TYPE:
	    case LBV_MOTION_SET_AOUT_TYPE:
	    case LBV_TRAJ_RIGID_TAP_TYPE:
	    case LBV_TRAJ_SET_TELEOP_ENABLE_TYPE:
	    case LBV_SET_DEBUG_TYPE:
		retval = lbvTaskIssueCommand(lbvCommand);
		break;

		// one case where we need to be in manual mode
	    case LBV_JOINT_OVERRIDE_LIMITS_TYPE:
		retval = 0;
		if (lbvStatus->task.mode == LBV_TASK_MODE_MANUAL) {
		    retval = lbvTaskIssueCommand(lbvCommand);
		}
		break;

	    case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
	    case LBV_TOOL_SET_OFFSET_TYPE:
		// send to IO
		lbvTaskQueueCommand(lbvCommand);
		// signify no more reading
		lbvTaskPlanSetWait();
		// then resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    case LBV_TOOL_SET_NUMBER_TYPE:
		// send to IO
		lbvTaskQueueCommand(lbvCommand);
		// then resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    default:
		lbvOperatorError(0,
				 _
				 ("command (%s) cannot be executed until the machine is out of E-stop and turned on"),
				 lbv_symbol_lookup(type));
		retval = -1;
		break;

	    }			// switch (type)

	default:
	    // invalid mode
	    break;

	}			// switch (mode)

	break;			// case LBV_TASK_STATE_OFF,ESTOP,ESTOP_RESET

    case LBV_TASK_STATE_ON:
	/* we can do everything (almost) when the machine is on, so let's
	   switch on the execution mode */
	switch (lbvStatus->task.mode) {
	case LBV_TASK_MODE_MANUAL:	// ON, MANUAL
	    switch (type) {
	    case 0:
	    case LBV_NULL_TYPE:
		// no command
		break;

		// immediate commands

	    case LBV_JOINT_DISABLE_TYPE:
	    case LBV_JOINT_ENABLE_TYPE:
	    case LBV_JOINT_SET_BACKLASH_TYPE:
	    case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
	    case LBV_JOINT_SET_FERROR_TYPE:
	    case LBV_JOINT_SET_MIN_FERROR_TYPE:
	    case LBV_JOINT_SET_MAX_POSITION_LIMIT_TYPE:
	    case LBV_JOINT_SET_MIN_POSITION_LIMIT_TYPE:
	    case LBV_JOINT_HALT_TYPE:
	    case LBV_JOINT_HOME_TYPE:
	    case LBV_JOINT_UNHOME_TYPE:
	    case LBV_JOG_CONT_TYPE:
	    case LBV_JOG_INCR_TYPE:
	    case LBV_JOG_ABS_TYPE:
	    case LBV_JOG_STOP_TYPE:
	    case LBV_JOINT_OVERRIDE_LIMITS_TYPE:
	    case LBV_TRAJ_PAUSE_TYPE:
	    case LBV_TRAJ_RESUME_TYPE:
	    case LBV_TRAJ_ABORT_TYPE:
	    case LBV_TRAJ_SET_SCALE_TYPE:
	    case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
	    case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
	    case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
	    case LBV_TRAJ_SET_FO_ENABLE_TYPE:
	    case LBV_TRAJ_SET_FH_ENABLE_TYPE:
	    case LBV_TRAJ_SET_SO_ENABLE_TYPE:
	    case LBV_SPINDLE_SPEED_TYPE:
	    case LBV_SPINDLE_ON_TYPE:
	    case LBV_SPINDLE_OFF_TYPE:
	    case LBV_SPINDLE_BRAKE_RELEASE_TYPE:
	    case LBV_SPINDLE_BRAKE_ENGAGE_TYPE:
	    case LBV_SPINDLE_INCREASE_TYPE:
	    case LBV_SPINDLE_DECREASE_TYPE:
	    case LBV_SPINDLE_CONSTANT_TYPE:
	    case LBV_COOLANT_MIST_ON_TYPE:
	    case LBV_COOLANT_MIST_OFF_TYPE:
	    case LBV_COOLANT_FLOOD_ON_TYPE:
	    case LBV_COOLANT_FLOOD_OFF_TYPE:
	    case LBV_LUBE_ON_TYPE:
	    case LBV_LUBE_OFF_TYPE:
	    case LBV_TASK_SET_MODE_TYPE:
	    case LBV_TASK_SET_STATE_TYPE:
	    case LBV_TASK_ABORT_TYPE:
	    case LBV_TASK_PLAN_OPEN_TYPE:
	    case LBV_TASK_PLAN_CLOSE_TYPE:
	    case LBV_TASK_PLAN_PAUSE_TYPE:
		case LBV_TASK_PLAN_REVERSE_TYPE:
		case LBV_TASK_PLAN_FORWARD_TYPE:
	    case LBV_TASK_PLAN_RESUME_TYPE:
	    case LBV_TASK_PLAN_INIT_TYPE:
	    case LBV_TASK_PLAN_SYNCH_TYPE:
	    case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	    case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	    case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
	    case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	    case LBV_TRAJ_PROBE_TYPE:
	    case LBV_AUX_INPUT_WAIT_TYPE:
	    case LBV_MOTION_SET_DOUT_TYPE:
	    case LBV_MOTION_SET_AOUT_TYPE:
	    case LBV_MOTION_ADAPTIVE_TYPE:
	    case LBV_TRAJ_RIGID_TAP_TYPE:
	    case LBV_TRAJ_SET_TELEOP_ENABLE_TYPE:
	    case LBV_SET_DEBUG_TYPE:
		retval = lbvTaskIssueCommand(lbvCommand);
		break;

		// queued commands

	    case LBV_TASK_PLAN_EXECUTE_TYPE:
		// resynch the interpreter, since we may have moved
		// externally
		lbvTaskIssueCommand(&taskPlanSynchCmd);
		// and now call for interpreter execute
		retval = lbvTaskIssueCommand(lbvCommand);
		break;

	    case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
	    case LBV_TOOL_SET_OFFSET_TYPE:
		// send to IO
		lbvTaskQueueCommand(lbvCommand);
		// signify no more reading
		lbvTaskPlanSetWait();
		// then resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    case LBV_TOOL_SET_NUMBER_TYPE:
		// send to IO
		lbvTaskQueueCommand(lbvCommand);
		// then resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
		break;

	    case LBV_TASK_PLAN_RUN_TYPE:
                if (GET_EXTERNAL_OFFSET_APPLIED()) {
                    // err here, fewer err reports
		    retval = -1;
		}
		break;

		// otherwise we can't handle it
	    default:
		lbvOperatorError(0, _("can't do that (%s:%d) in manual mode"),
				 lbv_symbol_lookup(type),(int) type);
		retval = -1;
		break;

	    }			// switch (type) in ON, MANUAL

	    break;		// case LBV_TASK_MODE_MANUAL

	case LBV_TASK_MODE_AUTO:	// ON, AUTO
	    switch (lbvStatus->task.interpState) {
	    case LBV_TASK_INTERP_IDLE:	// ON, AUTO, IDLE
		switch (type) {
		case 0:
		case LBV_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case LBV_JOINT_SET_BACKLASH_TYPE:
		case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
		case LBV_JOINT_SET_FERROR_TYPE:
		case LBV_JOINT_SET_MIN_FERROR_TYPE:
		case LBV_JOINT_UNHOME_TYPE:
		case LBV_TRAJ_PAUSE_TYPE:
		case LBV_TRAJ_RESUME_TYPE:
		case LBV_TRAJ_ABORT_TYPE:
		case LBV_TRAJ_SET_SCALE_TYPE:
		case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
		case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
		case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case LBV_TRAJ_SET_FO_ENABLE_TYPE:
        case LBV_TRAJ_SET_FH_ENABLE_TYPE:
		case LBV_TRAJ_SET_SO_ENABLE_TYPE:
		case LBV_SPINDLE_SPEED_TYPE:
		case LBV_SPINDLE_ORIENT_TYPE:
		case LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
		case LBV_SPINDLE_ON_TYPE:
		case LBV_SPINDLE_OFF_TYPE:
		case LBV_SPINDLE_BRAKE_RELEASE_TYPE:
		case LBV_SPINDLE_BRAKE_ENGAGE_TYPE:
		case LBV_SPINDLE_INCREASE_TYPE:
		case LBV_SPINDLE_DECREASE_TYPE:
		case LBV_SPINDLE_CONSTANT_TYPE:
		case LBV_COOLANT_MIST_ON_TYPE:
		case LBV_COOLANT_MIST_OFF_TYPE:
		case LBV_COOLANT_FLOOD_ON_TYPE:
		case LBV_COOLANT_FLOOD_OFF_TYPE:
		case LBV_LUBE_ON_TYPE:
		case LBV_LUBE_OFF_TYPE:
		case LBV_TASK_SET_MODE_TYPE:
		case LBV_TASK_SET_STATE_TYPE:
		case LBV_TASK_ABORT_TYPE:
		case LBV_TASK_PLAN_INIT_TYPE:
		case LBV_TASK_PLAN_OPEN_TYPE:
                case LBV_TASK_PLAN_CLOSE_TYPE:
		case LBV_TASK_PLAN_RUN_TYPE:
		case LBV_TASK_PLAN_EXECUTE_TYPE:
		case LBV_TASK_PLAN_PAUSE_TYPE:
		case LBV_TASK_PLAN_REVERSE_TYPE:
		case LBV_TASK_PLAN_FORWARD_TYPE:
		case LBV_TASK_PLAN_RESUME_TYPE:
		case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case LBV_TRAJ_PROBE_TYPE:
		case LBV_AUX_INPUT_WAIT_TYPE:
		case LBV_TRAJ_RIGID_TAP_TYPE:
		case LBV_SET_DEBUG_TYPE:
		    retval = lbvTaskIssueCommand(lbvCommand);
		    break;

		case LBV_TASK_PLAN_STEP_TYPE:
		    // handles case where first action is to step the program
		    taskPlanRunCmd.line = 1;	// run from start
		    /*! \todo FIXME-- can have GUI set this; send a run instead of a 
		       step */
		    retval = lbvTaskIssueCommand(&taskPlanRunCmd);
		    if(retval != 0) break;
		    lbvTrajPause();
		    if (lbvStatus->task.interpState != LBV_TASK_INTERP_PAUSED) {
			interpResumeState = lbvStatus->task.interpState;
		    }
		    lbvStatus->task.interpState = LBV_TASK_INTERP_PAUSED;
		    lbvStatus->task.task_paused = 1;
		    retval = 0;
		    break;

		case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
		case LBV_TOOL_SET_OFFSET_TYPE:
		    // send to IO
		    lbvTaskQueueCommand(lbvCommand);
		    // signify no more reading
		    lbvTaskPlanSetWait();
		    // then resynch interpreter
		    lbvTaskQueueCommand(&taskPlanSynchCmd);
		    break;
		    // otherwise we can't handle it
		default:
	            //LBV_TASK_MODE_AUTO(2) && LBV_TASK_INTERP_IDLE(1)
                    if ( allow_while_idle_type() ) {
                        retval = lbvTaskIssueCommand(lbvCommand);
		        break;
                    }
		    lbvOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter idle"),
			    lbv_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, IDLE

		break;		// LBV_TASK_INTERP_IDLE

	    case LBV_TASK_INTERP_READING:	// ON, AUTO, READING
		switch (type) {
		case 0:
		case LBV_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case LBV_JOINT_SET_BACKLASH_TYPE:
		case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
		case LBV_JOINT_SET_FERROR_TYPE:
		case LBV_JOINT_SET_MIN_FERROR_TYPE:
		case LBV_JOINT_UNHOME_TYPE:
		case LBV_TRAJ_PAUSE_TYPE:
		case LBV_TRAJ_RESUME_TYPE:
		case LBV_TRAJ_ABORT_TYPE:
		case LBV_TRAJ_SET_SCALE_TYPE:
		case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
                case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
		case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case LBV_TRAJ_SET_FO_ENABLE_TYPE:
		case LBV_TRAJ_SET_FH_ENABLE_TYPE:
		case LBV_TRAJ_SET_SO_ENABLE_TYPE:
		case LBV_SPINDLE_INCREASE_TYPE:
		case LBV_SPINDLE_DECREASE_TYPE:
		case LBV_SPINDLE_CONSTANT_TYPE:
		case LBV_TASK_PLAN_PAUSE_TYPE:
		case LBV_TASK_PLAN_REVERSE_TYPE:
		case LBV_TASK_PLAN_FORWARD_TYPE:
		case LBV_TASK_PLAN_RESUME_TYPE:
		case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case LBV_TASK_SET_MODE_TYPE:
		case LBV_TASK_SET_STATE_TYPE:
		case LBV_TASK_ABORT_TYPE:
		case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case LBV_TRAJ_PROBE_TYPE:
		case LBV_AUX_INPUT_WAIT_TYPE:
		case LBV_TRAJ_RIGID_TAP_TYPE:
		case LBV_SET_DEBUG_TYPE:
                case LBV_COOLANT_MIST_ON_TYPE:
                case LBV_COOLANT_MIST_OFF_TYPE:
                case LBV_COOLANT_FLOOD_ON_TYPE:
                case LBV_COOLANT_FLOOD_OFF_TYPE:
                case LBV_LUBE_ON_TYPE:
                case LBV_LUBE_OFF_TYPE:
		    retval = lbvTaskIssueCommand(lbvCommand);
		    return retval;
		    break;

		case LBV_TASK_PLAN_STEP_TYPE:
		    stepping = 1;	// set stepping mode in case it's not
		    steppingWait = 0;	// clear the wait
		    break;

		    // otherwise we can't handle it
		default:
		    lbvOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter reading"),
			    lbv_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, READING

               // handle interp readahead logic
                readahead_reading();
                
		break;		// LBV_TASK_INTERP_READING

	    case LBV_TASK_INTERP_PAUSED:	// ON, AUTO, PAUSED
		switch (type) {
		case 0:
		case LBV_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case LBV_JOINT_SET_BACKLASH_TYPE:
		case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
		case LBV_JOINT_SET_FERROR_TYPE:
		case LBV_JOINT_SET_MIN_FERROR_TYPE:
		case LBV_JOINT_UNHOME_TYPE:
		case LBV_TRAJ_PAUSE_TYPE:
		case LBV_TRAJ_RESUME_TYPE:
		case LBV_TRAJ_ABORT_TYPE:
		case LBV_TRAJ_SET_SCALE_TYPE:
		case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
		case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
		case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case LBV_TRAJ_SET_FO_ENABLE_TYPE:
	        case LBV_TRAJ_SET_FH_ENABLE_TYPE:
		case LBV_TRAJ_SET_SO_ENABLE_TYPE:
		case LBV_SPINDLE_SPEED_TYPE:
		case LBV_SPINDLE_ORIENT_TYPE:
		case LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
		case LBV_SPINDLE_ON_TYPE:
		case LBV_SPINDLE_OFF_TYPE:
		case LBV_SPINDLE_BRAKE_RELEASE_TYPE:
		case LBV_SPINDLE_BRAKE_ENGAGE_TYPE:
		case LBV_SPINDLE_INCREASE_TYPE:
		case LBV_SPINDLE_DECREASE_TYPE:
		case LBV_SPINDLE_CONSTANT_TYPE:
		case LBV_COOLANT_MIST_ON_TYPE:
		case LBV_COOLANT_MIST_OFF_TYPE:
		case LBV_COOLANT_FLOOD_ON_TYPE:
		case LBV_COOLANT_FLOOD_OFF_TYPE:
		case LBV_LUBE_ON_TYPE:
		case LBV_LUBE_OFF_TYPE:
		case LBV_TASK_SET_MODE_TYPE:
		case LBV_TASK_SET_STATE_TYPE:
		case LBV_TASK_ABORT_TYPE:
		case LBV_TASK_PLAN_EXECUTE_TYPE:
		case LBV_TASK_PLAN_PAUSE_TYPE:
		case LBV_TASK_PLAN_REVERSE_TYPE:
		case LBV_TASK_PLAN_FORWARD_TYPE:
		case LBV_TASK_PLAN_RESUME_TYPE:
		case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case LBV_TRAJ_PROBE_TYPE:
		case LBV_AUX_INPUT_WAIT_TYPE:
		case LBV_TRAJ_RIGID_TAP_TYPE:
		case LBV_SET_DEBUG_TYPE:
		    retval = lbvTaskIssueCommand(lbvCommand);
		    break;

		case LBV_TASK_PLAN_STEP_TYPE:
		    stepping = 1;
		    steppingWait = 0;
		    if (lbvStatus->motion.traj.paused &&
			lbvStatus->motion.traj.queue > 0) {
			// there are pending motions paused; step them
			lbvTrajStep();
		    } else {
			lbvStatus->task.interpState = (enum LBV_TASK_INTERP_ENUM) interpResumeState;
		    }
		    lbvStatus->task.task_paused = 1;
		    break;

		    // otherwise we can't handle it
		default:
		    lbvOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter paused"),
			    lbv_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, PAUSED

		break;		// LBV_TASK_INTERP_PAUSED

	    case LBV_TASK_INTERP_WAITING:
		// interpreter ran to end
		// handle input commands
		switch (type) {
		case 0:
		case LBV_NULL_TYPE:
		    // no command
		    break;

		    // immediate commands

		case LBV_JOINT_SET_BACKLASH_TYPE:
		case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
		case LBV_JOINT_SET_FERROR_TYPE:
		case LBV_JOINT_SET_MIN_FERROR_TYPE:
		case LBV_JOINT_UNHOME_TYPE:
		case LBV_TRAJ_PAUSE_TYPE:
		case LBV_TRAJ_RESUME_TYPE:
		case LBV_TRAJ_ABORT_TYPE:
		case LBV_TRAJ_SET_SCALE_TYPE:
		case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
		case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
		case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
		case LBV_TRAJ_SET_FO_ENABLE_TYPE:
	        case LBV_TRAJ_SET_FH_ENABLE_TYPE:
		case LBV_TRAJ_SET_SO_ENABLE_TYPE:
		case LBV_SPINDLE_INCREASE_TYPE:
		case LBV_SPINDLE_DECREASE_TYPE:
		case LBV_SPINDLE_CONSTANT_TYPE:
		case LBV_TASK_PLAN_EXECUTE_TYPE:
		case LBV_TASK_PLAN_PAUSE_TYPE:
		case LBV_TASK_PLAN_REVERSE_TYPE:
		case LBV_TASK_PLAN_FORWARD_TYPE:
		case LBV_TASK_PLAN_RESUME_TYPE:
		case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
		case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
		case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
		case LBV_TASK_SET_MODE_TYPE:
		case LBV_TASK_SET_STATE_TYPE:
		case LBV_TASK_ABORT_TYPE:
		case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
		case LBV_TRAJ_PROBE_TYPE:
		case LBV_AUX_INPUT_WAIT_TYPE:
	        case LBV_TRAJ_RIGID_TAP_TYPE:
		case LBV_SET_DEBUG_TYPE:
                case LBV_COOLANT_MIST_ON_TYPE:
                case LBV_COOLANT_MIST_OFF_TYPE:
                case LBV_COOLANT_FLOOD_ON_TYPE:
                case LBV_COOLANT_FLOOD_OFF_TYPE:
                case LBV_LUBE_ON_TYPE:
                case LBV_LUBE_OFF_TYPE:
		    retval = lbvTaskIssueCommand(lbvCommand);
		    break;

		case LBV_TASK_PLAN_STEP_TYPE:
		    stepping = 1;	// set stepping mode in case it's not
		    steppingWait = 0;	// clear the wait
		    break;

		    // otherwise we can't handle it
		default:
		    lbvOperatorError(0, _
			    ("can't do that (%s) in auto mode with the interpreter waiting"),
			    lbv_symbol_lookup(type));
		    retval = -1;
		    break;

		}		// switch (type) in ON, AUTO, WAITING

                // handle interp readahead logic
                readahead_waiting();

		break;		// end of case LBV_TASK_INTERP_WAITING

	    default:
		// coding error
		rcs_print_error("invalid mode(%d)", lbvStatus->task.mode);
		retval = -1;
		break;

	    }			// switch (mode) in ON, AUTO

	    break;		// case LBV_TASK_MODE_AUTO

	case LBV_TASK_MODE_MDI:	// ON, MDI
	    switch (type) {
	    case 0:
	    case LBV_NULL_TYPE:
		// no command
		break;

		// immediate commands

	    case LBV_JOINT_SET_BACKLASH_TYPE:
	    case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
	    case LBV_JOINT_SET_FERROR_TYPE:
	    case LBV_JOINT_SET_MIN_FERROR_TYPE:
	    case LBV_JOINT_UNHOME_TYPE:
	    case LBV_TRAJ_SET_SCALE_TYPE:
	    case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
	    case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
	    case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
	    case LBV_TRAJ_SET_FO_ENABLE_TYPE:
	    case LBV_TRAJ_SET_FH_ENABLE_TYPE:
	    case LBV_TRAJ_SET_SO_ENABLE_TYPE:
	    case LBV_SPINDLE_SPEED_TYPE:
	    case LBV_SPINDLE_ORIENT_TYPE:
	    case LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	    case LBV_SPINDLE_ON_TYPE:
	    case LBV_SPINDLE_OFF_TYPE:
	    case LBV_SPINDLE_BRAKE_RELEASE_TYPE:
	    case LBV_SPINDLE_BRAKE_ENGAGE_TYPE:
	    case LBV_SPINDLE_INCREASE_TYPE:
	    case LBV_SPINDLE_DECREASE_TYPE:
	    case LBV_SPINDLE_CONSTANT_TYPE:
	    case LBV_COOLANT_MIST_ON_TYPE:
	    case LBV_COOLANT_MIST_OFF_TYPE:
	    case LBV_COOLANT_FLOOD_ON_TYPE:
	    case LBV_COOLANT_FLOOD_OFF_TYPE:
	    case LBV_LUBE_ON_TYPE:
	    case LBV_LUBE_OFF_TYPE:
	    case LBV_TASK_SET_MODE_TYPE:
	    case LBV_TASK_SET_STATE_TYPE:
	    case LBV_TASK_PLAN_INIT_TYPE:
	    case LBV_TASK_PLAN_OPEN_TYPE:
	    case LBV_TASK_PLAN_CLOSE_TYPE:
	    case LBV_TASK_PLAN_PAUSE_TYPE:
		case LBV_TASK_PLAN_REVERSE_TYPE:
		case LBV_TASK_PLAN_FORWARD_TYPE:
	    case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	    case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	    case LBV_TASK_PLAN_RESUME_TYPE:
	    case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
	    case LBV_TASK_PLAN_SYNCH_TYPE:
	    case LBV_TASK_ABORT_TYPE:
	    case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	    case LBV_TRAJ_PROBE_TYPE:
	    case LBV_AUX_INPUT_WAIT_TYPE:
	    case LBV_MOTION_SET_DOUT_TYPE:
	    case LBV_MOTION_SET_AOUT_TYPE:
	    case LBV_MOTION_ADAPTIVE_TYPE:
	    case LBV_TRAJ_RIGID_TAP_TYPE:
	    case LBV_SET_DEBUG_TYPE:
		retval = lbvTaskIssueCommand(lbvCommand);
		break;

	    case LBV_TASK_PLAN_EXECUTE_TYPE:
                // If there are no queued MDI commands, no commands in
                // interp_list, and waitFlag isn't set, then this new
                // incoming MDI command can just be issued directly.
                // Otherwise we need to queue it and deal with it
                // later.
                if (
                    (mdi_execute_queue.len() == 0)
                    && (interp_list.len() == 0)
                    && (lbvTaskCommand == NULL)
		    && (lbvTaskPlanIsWait() == 0)
                ) {
                    retval = lbvTaskIssueCommand(lbvCommand);
                } else {
                    mdi_execute_queue.append(lbvCommand);
                    lbvStatus->task.queuedMDIcommands = mdi_execute_queue.len();
                    retval = 0;
                }
                break;

	    case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
	    case LBV_TOOL_SET_OFFSET_TYPE:
		// send to IO
		lbvTaskQueueCommand(lbvCommand);
		// signify no more reading
		lbvTaskPlanSetWait();
		// then resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
		break;

		// otherwise we can't handle it
	    default:
	        //LBV_TASK_MODE_MDI(3) && LBV_TASK_INTERP_IDLE(1)
                if ( allow_while_idle_type() ) {
                    retval = lbvTaskIssueCommand(lbvCommand);
		    break;
                }
		lbvOperatorError(0, _("can't do that (%s:%d) in MDI mode"),
			lbv_symbol_lookup(type),(int) type);

		retval = -1;
		break;

	    }			// switch (type) in ON, MDI
	    mdi_execute_hook();

	    break;		// case LBV_TASK_MODE_MDI

	default:
	    break;

	}			// switch (mode)

	break;			// case LBV_TASK_STATE_ON

    default:
	break;

    }				// switch (task.state)

    return retval;
}

/*
   lbvTaskCheckPreconditions() is called for commands on the interp_list.
   Immediate commands, i.e., commands sent from calls to lbvTaskIssueCommand()
   in lbvTaskPlan() directly, are not handled here.

   The return value is a state for lbvTaskExecute() to wait on, e.g.,
   LBV_TASK_EXEC_WAITING_FOR_MOTION, before the command can be sent out.
   */
static int lbvTaskCheckPreconditions(NMLmsg * cmd)
{
    if (0 == cmd) {
	return LBV_TASK_EXEC_DONE;
    }

    switch (cmd->type) {
	// operator messages, if queued, will go out when everything before
	// them is done
    case LBV_OPERATOR_ERROR_TYPE:
    case LBV_OPERATOR_TEXT_TYPE:
    case LBV_OPERATOR_DISPLAY_TYPE:
    case LBV_SYSTEM_CMD_TYPE:
    case LBV_TRAJ_PROBE_TYPE:	// prevent blending of this
    case LBV_TRAJ_RIGID_TAP_TYPE: //and this
    case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:	// and this
    case LBV_AUX_INPUT_WAIT_TYPE:
    case LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_TRAJ_LINEAR_MOVE_TYPE:
    case LBV_TRAJ_CIRCULAR_MOVE_TYPE:
    case LBV_TRAJ_SET_VELOCITY_TYPE:
    case LBV_TRAJ_SET_ACCELERATION_TYPE:
    case LBV_TRAJ_SET_TERM_COND_TYPE:
    case LBV_TRAJ_SET_SPINDLESYNC_TYPE:
    case LBV_TRAJ_SET_FO_ENABLE_TYPE:
    case LBV_TRAJ_SET_FH_ENABLE_TYPE:
    case LBV_TRAJ_SET_SO_ENABLE_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_IO;
	break;

    case LBV_TRAJ_SET_OFFSET_TYPE:
	// this applies the tool length offset variable after previous
	// motions
    case LBV_TRAJ_SET_G5X_TYPE:
    case LBV_TRAJ_SET_G92_TYPE:
    case LBV_TRAJ_SET_ROTATION_TYPE:
	// this applies the program origin after previous motions
	return LBV_TASK_EXEC_WAITING_FOR_MOTION;
	break;

    case LBV_TOOL_LOAD_TYPE:
    case LBV_TOOL_UNLOAD_TYPE:
    case LBV_TOOL_START_CHANGE_TYPE:
    case LBV_COOLANT_MIST_ON_TYPE:
    case LBV_COOLANT_MIST_OFF_TYPE:
    case LBV_COOLANT_FLOOD_ON_TYPE:
    case LBV_COOLANT_FLOOD_OFF_TYPE:
    case LBV_SPINDLE_SPEED_TYPE:
    case LBV_SPINDLE_ON_TYPE:
    case LBV_SPINDLE_OFF_TYPE:
    case LBV_SPINDLE_ORIENT_TYPE: // not sure
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_TOOL_PREPARE_TYPE:
    case LBV_LUBE_ON_TYPE:
    case LBV_LUBE_OFF_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_IO;
	break;

    case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
    case LBV_TOOL_SET_OFFSET_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_TOOL_SET_NUMBER_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_IO;
	break;

    case LBV_TASK_PLAN_PAUSE_TYPE:
    case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
	/* pause on the interp list is queued, so wait until all are done */
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_TASK_PLAN_END_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_TASK_PLAN_INIT_TYPE:
    case LBV_TASK_PLAN_RUN_TYPE:
    case LBV_TASK_PLAN_SYNCH_TYPE:
    case LBV_TASK_PLAN_EXECUTE_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_TRAJ_DELAY_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO;
	break;

    case LBV_MOTION_SET_AOUT_TYPE:
	if (((LBV_MOTION_SET_AOUT *) cmd)->now) {
    	    return LBV_TASK_EXEC_WAITING_FOR_MOTION;
	}
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_MOTION_SET_DOUT_TYPE:
	if (((LBV_MOTION_SET_DOUT *) cmd)->now) {
    	    return LBV_TASK_EXEC_WAITING_FOR_MOTION;
	}
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_MOTION_ADAPTIVE_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_MOTION;
	break;

    case LBV_EXEC_PLUGIN_CALL_TYPE:
    case LBV_IO_PLUGIN_CALL_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;


    default:
	// unrecognized command
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print_error("preconditions: unrecognized command %d:%s\n",
			    (int)cmd->type, lbv_symbol_lookup(cmd->type));
	}
	return LBV_TASK_EXEC_ERROR;
	break;
    }

    return LBV_TASK_EXEC_DONE;
}

// puts command on interp list
int lbvTaskQueueCommand(NMLmsg * cmd)
{
    if (0 == cmd) {
	return 0;
    }
    interp_list.append(cmd);

    return 0;
}

// issues command immediately
static int lbvTaskIssueCommand(NMLmsg * cmd)
{
    int retval = 0;
    int execRetval = 0;

    if (0 == cmd) {
        if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
            rcs_print("lbvTaskIssueCommand() null command\n");
        }
	return 0;
    }
    if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	rcs_print("Issuing %s -- \t (%s)\n", lbvSymbolLookup(cmd->type),
		  lbvCommandBuffer->msg2str(cmd));
    }
    switch (cmd->type) {
	// general commands

    case LBV_OPERATOR_ERROR_TYPE:
	retval = lbvOperatorError(((LBV_OPERATOR_ERROR *) cmd)->id,
				  "%s", ((LBV_OPERATOR_ERROR *) cmd)->error);
	break;

    case LBV_OPERATOR_TEXT_TYPE:
	retval = lbvOperatorText(((LBV_OPERATOR_TEXT *) cmd)->id,
				 "%s", ((LBV_OPERATOR_TEXT *) cmd)->text);
	break;

    case LBV_OPERATOR_DISPLAY_TYPE:
	retval = lbvOperatorDisplay(((LBV_OPERATOR_DISPLAY *) cmd)->id,
				    "%s", ((LBV_OPERATOR_DISPLAY *) cmd)->
				    display);
	break;

    case LBV_SYSTEM_CMD_TYPE:
	retval = lbvSystemCmd(((LBV_SYSTEM_CMD *) cmd)->string);
	break;

	// joint commands

    case LBV_JOINT_DISABLE_TYPE:
	disable_msg = (LBV_JOINT_DISABLE *) cmd;
	retval = lbvJointDisable(disable_msg->joint);
	break;

    case LBV_JOINT_ENABLE_TYPE:
	enable_msg = (LBV_JOINT_ENABLE *) cmd;
	retval = lbvJointEnable(enable_msg->joint);
	break;

    case LBV_JOINT_HOME_TYPE:
	home_msg = (LBV_JOINT_HOME *) cmd;
	retval = lbvJointHome(home_msg->joint);
	break;

    case LBV_JOINT_UNHOME_TYPE:
	unhome_msg = (LBV_JOINT_UNHOME *) cmd;
	retval = lbvJointUnhome(unhome_msg->joint);
	break;

    case LBV_JOG_CONT_TYPE:
	jog_cont_msg = (LBV_JOG_CONT *) cmd;
	retval = lbvJogCont(jog_cont_msg->joint_or_axis,
                            jog_cont_msg->vel,
                            jog_cont_msg->jjogmode);
	break;

    case LBV_JOG_STOP_TYPE:
	jog_stop_msg = (LBV_JOG_STOP *) cmd;
	retval = lbvJogStop(jog_stop_msg->joint_or_axis,
                            jog_stop_msg->jjogmode);
	break;

    case LBV_JOG_INCR_TYPE:
	jog_incr_msg = (LBV_JOG_INCR *) cmd;
	retval = lbvJogIncr(jog_incr_msg->joint_or_axis,
			    jog_incr_msg->incr,
                            jog_incr_msg->vel,
                            jog_incr_msg->jjogmode);
	break;

    case LBV_JOG_ABS_TYPE:
	jog_abs_msg = (LBV_JOG_ABS *) cmd;
	retval = lbvJogAbs(jog_abs_msg->joint_or_axis,
	                   jog_abs_msg->pos,
                           jog_abs_msg->vel,
                           jog_abs_msg->jjogmode);
	break;

    case LBV_JOINT_SET_BACKLASH_TYPE:
	set_backlash_msg = (LBV_JOINT_SET_BACKLASH *) cmd;
	retval =
	    lbvJointSetBacklash(set_backlash_msg->joint,
			       set_backlash_msg->backlash);
	break;

    case LBV_JOINT_SET_HOMING_PARAMS_TYPE:
	set_homing_params_msg = (LBV_JOINT_SET_HOMING_PARAMS *) cmd;
	retval = lbvJointSetHomingParams(set_homing_params_msg->joint,
					set_homing_params_msg->home,
					set_homing_params_msg->offset,
					set_homing_params_msg->home_final_vel,
					set_homing_params_msg->search_vel,
					set_homing_params_msg->latch_vel,
					set_homing_params_msg->use_index,
					set_homing_params_msg->encoder_does_not_reset,
					set_homing_params_msg->ignore_limits,
					set_homing_params_msg->is_shared,
					set_homing_params_msg->home_sequence,
					set_homing_params_msg->volatile_home,
					set_homing_params_msg->locking_indexer,
					set_homing_params_msg->absolute_encoder);
	break;

    case LBV_JOINT_SET_FERROR_TYPE:
	set_ferror_msg = (LBV_JOINT_SET_FERROR *) cmd;
	retval = lbvJointSetFerror(set_ferror_msg->joint,
				  set_ferror_msg->ferror);
	break;

    case LBV_JOINT_SET_MIN_FERROR_TYPE:
	set_min_ferror_msg = (LBV_JOINT_SET_MIN_FERROR *) cmd;
	retval = lbvJointSetMinFerror(set_min_ferror_msg->joint,
				     set_min_ferror_msg->ferror);
	break;

    case LBV_JOINT_SET_MAX_POSITION_LIMIT_TYPE:
	set_max_limit_msg = (LBV_JOINT_SET_MAX_POSITION_LIMIT *) cmd;
	retval = lbvJointSetMaxPositionLimit(set_max_limit_msg->joint,
					    set_max_limit_msg->limit);
	break;

    case LBV_JOINT_SET_MIN_POSITION_LIMIT_TYPE:
	set_min_limit_msg = (LBV_JOINT_SET_MIN_POSITION_LIMIT *) cmd;
	retval = lbvJointSetMinPositionLimit(set_min_limit_msg->joint,
					    set_min_limit_msg->limit);
	break;

    case LBV_JOINT_HALT_TYPE:
	joint_halt_msg = (LBV_JOINT_HALT *) cmd;
	retval = lbvJointHalt(joint_halt_msg->joint);
	break;

    case LBV_JOINT_OVERRIDE_LIMITS_TYPE:
	joint_lim_msg = (LBV_JOINT_OVERRIDE_LIMITS *) cmd;
	retval = lbvJointOverrideLimits(joint_lim_msg->joint);
	break;

    case LBV_JOINT_LOAD_COMP_TYPE:
	joint_load_comp_msg = (LBV_JOINT_LOAD_COMP *) cmd;
	retval = lbvJointLoadComp(joint_load_comp_msg->joint,
				 joint_load_comp_msg->file,
				 joint_load_comp_msg->type);
	break;

	// traj commands

    case LBV_TRAJ_SET_SCALE_TYPE:
	lbvTrajSetScaleMsg = (LBV_TRAJ_SET_SCALE *) cmd;
	retval = lbvTrajSetScale(lbvTrajSetScaleMsg->scale);
	break;

    case LBV_TRAJ_SET_RAPID_SCALE_TYPE:
	lbvTrajSetRapidScaleMsg = (LBV_TRAJ_SET_RAPID_SCALE *) cmd;
	retval = lbvTrajSetRapidScale(lbvTrajSetRapidScaleMsg->scale);
	break;

    case LBV_TRAJ_SET_MAX_VELOCITY_TYPE:
	lbvTrajSetMaxVelocityMsg = (LBV_TRAJ_SET_MAX_VELOCITY *) cmd;
	retval = lbvTrajSetMaxVelocity(lbvTrajSetMaxVelocityMsg->velocity);
	break;

    case LBV_TRAJ_SET_SPINDLE_SCALE_TYPE:
	lbvTrajSetSpindleScaleMsg = (LBV_TRAJ_SET_SPINDLE_SCALE *) cmd;
	retval = lbvTrajSetSpindleScale(lbvTrajSetSpindleScaleMsg->spindle,
                                    lbvTrajSetSpindleScaleMsg->scale);
	break;

    case LBV_TRAJ_SET_FO_ENABLE_TYPE:
	retval = lbvTrajSetFOEnable(((LBV_TRAJ_SET_FO_ENABLE *) cmd)->mode);  // feed override enable/disable
	break;

    case LBV_TRAJ_SET_FH_ENABLE_TYPE:
	retval = lbvTrajSetFHEnable(((LBV_TRAJ_SET_FH_ENABLE *) cmd)->mode); //feed hold enable/disable
	break;

    case LBV_TRAJ_SET_SO_ENABLE_TYPE:
	retval = lbvTrajSetSOEnable(((LBV_TRAJ_SET_SO_ENABLE *) cmd)->mode); //spindle speed override enable/disable
	break;

    case LBV_TRAJ_SET_VELOCITY_TYPE:
	lbvTrajSetVelocityMsg = (LBV_TRAJ_SET_VELOCITY *) cmd;
	retval = lbvTrajSetVelocity(lbvTrajSetVelocityMsg->velocity,
			lbvTrajSetVelocityMsg->ini_maxvel);
	break;

    case LBV_TRAJ_SET_ACCELERATION_TYPE:
	lbvTrajSetAccelerationMsg = (LBV_TRAJ_SET_ACCELERATION *) cmd;
	retval = lbvTrajSetAcceleration(lbvTrajSetAccelerationMsg->acceleration);
	break;

    case LBV_TRAJ_LINEAR_MOVE_TYPE:
	lbvTrajLinearMoveMsg = (LBV_TRAJ_LINEAR_MOVE *) cmd;
        retval = lbvTrajLinearMove(lbvTrajLinearMoveMsg->end,
                                   lbvTrajLinearMoveMsg->type, lbvTrajLinearMoveMsg->vel,
                                   lbvTrajLinearMoveMsg->ini_maxvel, lbvTrajLinearMoveMsg->acc,
                                   lbvTrajLinearMoveMsg->indexer_jnum);
	break;

    case LBV_TRAJ_CIRCULAR_MOVE_TYPE:
	lbvTrajCircularMoveMsg = (LBV_TRAJ_CIRCULAR_MOVE *) cmd;
        retval = lbvTrajCircularMove(lbvTrajCircularMoveMsg->end,
                lbvTrajCircularMoveMsg->center, lbvTrajCircularMoveMsg->normal,
                lbvTrajCircularMoveMsg->turn, lbvTrajCircularMoveMsg->type,
                lbvTrajCircularMoveMsg->vel,
                lbvTrajCircularMoveMsg->ini_maxvel,
                lbvTrajCircularMoveMsg->acc);
	break;

    case LBV_TRAJ_PAUSE_TYPE:
	lbvStatus->task.task_paused = 1;
	retval = lbvTrajPause();
	break;

    case LBV_TRAJ_RESUME_TYPE:
	lbvStatus->task.task_paused = 0;
	retval = lbvTrajResume();
	break;

    case LBV_TRAJ_ABORT_TYPE:
	retval = lbvTrajAbort();
	break;

    case LBV_TRAJ_DELAY_TYPE:
	lbvTrajDelayMsg = (LBV_TRAJ_DELAY *) cmd;
	// set the timeout clock to expire at 'now' + delay time
	taskExecDelayTimeout = etime() + lbvTrajDelayMsg->delay;
	retval = 0;
	break;

    case LBV_TRAJ_SET_TERM_COND_TYPE:
	lbvTrajSetTermCondMsg = (LBV_TRAJ_SET_TERM_COND *) cmd;
	retval = lbvTrajSetTermCond(lbvTrajSetTermCondMsg->cond, lbvTrajSetTermCondMsg->tolerance);
	break;

    case LBV_TRAJ_SET_SPINDLESYNC_TYPE:
        lbvTrajSetSpindlesyncMsg = (LBV_TRAJ_SET_SPINDLESYNC *) cmd;
        retval = lbvTrajSetSpindleSync(lbvTrajSetSpindlesyncMsg->spindle, lbvTrajSetSpindlesyncMsg->feed_per_revolution, lbvTrajSetSpindlesyncMsg->velocity_mode);
        break;

    case LBV_TRAJ_SET_OFFSET_TYPE:
	// update tool offset
	lbvStatus->task.toolOffset = ((LBV_TRAJ_SET_OFFSET *) cmd)->offset;
        retval = lbvTrajSetOffset(lbvStatus->task.toolOffset);
	break;

    case LBV_TRAJ_SET_ROTATION_TYPE:
        lbvStatus->task.rotation_xy = ((LBV_TRAJ_SET_ROTATION *) cmd)->rotation;
        retval = 0;
        break;

    case LBV_TRAJ_SET_G5X_TYPE:
	// struct-copy program origin
	lbvStatus->task.g5x_offset = ((LBV_TRAJ_SET_G5X *) cmd)->origin;
        lbvStatus->task.g5x_index = ((LBV_TRAJ_SET_G5X *) cmd)->g5x_index;
	retval = 0;
	break;
    case LBV_TRAJ_SET_G92_TYPE:
	// struct-copy program origin
	lbvStatus->task.g92_offset = ((LBV_TRAJ_SET_G92 *) cmd)->origin;
	retval = 0;
	break;
    case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
	retval = lbvTrajClearProbeTrippedFlag();
	break;

    case LBV_TRAJ_PROBE_TYPE:
	retval = lbvTrajProbe(
	    ((LBV_TRAJ_PROBE *) cmd)->pos, 
	    ((LBV_TRAJ_PROBE *) cmd)->type,
	    ((LBV_TRAJ_PROBE *) cmd)->vel,
            ((LBV_TRAJ_PROBE *) cmd)->ini_maxvel,  
	    ((LBV_TRAJ_PROBE *) cmd)->acc,
            ((LBV_TRAJ_PROBE *) cmd)->probe_type);
	break;

    case LBV_AUX_INPUT_WAIT_TYPE:
	lbvAuxInputWaitMsg = (LBV_AUX_INPUT_WAIT *) cmd;
	if (lbvAuxInputWaitMsg->timeout == WAIT_MODE_IMMEDIATE) { //nothing to do, CANON will get the needed value when asked by the interp
	    lbvStatus->task.input_timeout = 0; // no timeout can occur
	    lbvAuxInputWaitIndex = -1;
	    taskExecDelayTimeout = 0.0;
	} else {
	    lbvAuxInputWaitType = lbvAuxInputWaitMsg->wait_type; // remember what we are waiting for 
	    lbvAuxInputWaitIndex = lbvAuxInputWaitMsg->index; // remember the input to look at
	    lbvStatus->task.input_timeout = 2; // set timeout flag, gets cleared if input changes before timeout happens
	    // set the timeout clock to expire at 'now' + delay time
	    taskExecDelayTimeout = etime() + lbvAuxInputWaitMsg->timeout;
	}
	break;

    case LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	wait_spindle_orient_complete_msg = (LBV_SPINDLE_WAIT_ORIENT_COMPLETE *) cmd;
	taskExecDelayTimeout = etime() + wait_spindle_orient_complete_msg->timeout;
	break;

    case LBV_TRAJ_RIGID_TAP_TYPE:
	retval = lbvTrajRigidTap(((LBV_TRAJ_RIGID_TAP *) cmd)->pos,
	        ((LBV_TRAJ_RIGID_TAP *) cmd)->vel,
        	((LBV_TRAJ_RIGID_TAP *) cmd)->ini_maxvel,  
		((LBV_TRAJ_RIGID_TAP *) cmd)->acc,
		((LBV_TRAJ_RIGID_TAP *) cmd)->scale);
	break;

    case LBV_TRAJ_SET_TELEOP_ENABLE_TYPE:
	if (((LBV_TRAJ_SET_TELEOP_ENABLE *) cmd)->enable) {
	    retval = lbvTrajSetMode(LBV_TRAJ_MODE_TELEOP);
	} else {
	    retval = lbvTrajSetMode(LBV_TRAJ_MODE_FREE);
	}
	break;

    case LBV_MOTION_SET_AOUT_TYPE:
	retval = lbvMotionSetAout(((LBV_MOTION_SET_AOUT *) cmd)->index,
				  ((LBV_MOTION_SET_AOUT *) cmd)->start,
				  ((LBV_MOTION_SET_AOUT *) cmd)->end,
				  ((LBV_MOTION_SET_AOUT *) cmd)->now);
	break;

    case LBV_MOTION_SET_DOUT_TYPE:
	retval = lbvMotionSetDout(((LBV_MOTION_SET_DOUT *) cmd)->index,
				  ((LBV_MOTION_SET_DOUT *) cmd)->start,
				  ((LBV_MOTION_SET_DOUT *) cmd)->end,
				  ((LBV_MOTION_SET_DOUT *) cmd)->now);
	break;

    case LBV_MOTION_ADAPTIVE_TYPE:
	retval = lbvTrajSetAFEnable(((LBV_MOTION_ADAPTIVE *) cmd)->status);
	break;

    case LBV_SET_DEBUG_TYPE:
	/* set the debug level here */
	lbv_debug = ((LBV_SET_DEBUG *) cmd)->debug;
	/* and in IO and motion */
	lbvIoSetDebug(lbv_debug);
	lbvMotionSetDebug(lbv_debug);
	/* and reflect it in the status-- this isn't updated continually */
	lbvStatus->debug = lbv_debug;
	break;

	// unimplemented ones

	// IO commands

    case LBV_SPINDLE_SPEED_TYPE:
	spindle_speed_msg = (LBV_SPINDLE_SPEED *) cmd;
	retval = lbvSpindleSpeed(spindle_speed_msg->spindle, spindle_speed_msg->speed,
			spindle_speed_msg->factor, spindle_speed_msg->xoffset);
	break;

    case LBV_SPINDLE_ORIENT_TYPE:
	spindle_orient_msg = (LBV_SPINDLE_ORIENT *) cmd;
	retval = lbvSpindleOrient(spindle_orient_msg->spindle, spindle_orient_msg->orientation,
			spindle_orient_msg->mode);
	break;

    case LBV_SPINDLE_ON_TYPE:
	spindle_on_msg = (LBV_SPINDLE_ON *) cmd;
	retval = lbvSpindleOn(spindle_on_msg->spindle, spindle_on_msg->speed,
			spindle_on_msg->factor, spindle_on_msg->xoffset, spindle_on_msg->wait_for_spindle_at_speed);
	break;

    case LBV_SPINDLE_OFF_TYPE:
    spindle_off_msg = (LBV_SPINDLE_OFF *) cmd;
	retval = lbvSpindleOff(spindle_off_msg->spindle);
	break;

    case LBV_SPINDLE_BRAKE_RELEASE_TYPE:
    spindle_brake_release_msg = (LBV_SPINDLE_BRAKE_RELEASE *) cmd;
	retval = lbvSpindleBrakeRelease(spindle_brake_release_msg->spindle);
	break;

    case LBV_SPINDLE_INCREASE_TYPE:
    spindle_increase_msg = (LBV_SPINDLE_INCREASE *) cmd;
	retval = lbvSpindleIncrease(spindle_increase_msg->spindle);
	break;

    case LBV_SPINDLE_DECREASE_TYPE:
    spindle_decrease_msg = (LBV_SPINDLE_DECREASE *) cmd;
    retval = lbvSpindleDecrease(spindle_decrease_msg->spindle);
	break;

    case LBV_SPINDLE_CONSTANT_TYPE:
    spindle_constant_msg = (LBV_SPINDLE_CONSTANT *) cmd;
    retval = lbvSpindleConstant(spindle_constant_msg->spindle);
	break;

    case LBV_SPINDLE_BRAKE_ENGAGE_TYPE:
    spindle_brake_engage_msg = (LBV_SPINDLE_BRAKE_ENGAGE *) cmd;
    retval = lbvSpindleBrakeEngage(spindle_brake_engage_msg->spindle);
	break;

    case LBV_COOLANT_MIST_ON_TYPE:
	retval = lbvCoolantMistOn();
	break;

    case LBV_COOLANT_MIST_OFF_TYPE:
	retval = lbvCoolantMistOff();
	break;

    case LBV_COOLANT_FLOOD_ON_TYPE:
	retval = lbvCoolantFloodOn();
	break;

    case LBV_COOLANT_FLOOD_OFF_TYPE:
	retval = lbvCoolantFloodOff();
	break;

    case LBV_LUBE_ON_TYPE:
	retval = lbvLubeOn();
	break;

    case LBV_LUBE_OFF_TYPE:
	retval = lbvLubeOff();
	break;

    case LBV_TOOL_PREPARE_TYPE:
	tool_prepare_msg = (LBV_TOOL_PREPARE *) cmd;
	retval = lbvToolPrepare(tool_prepare_msg->pocket,tool_prepare_msg->tool);
	break;

    case LBV_TOOL_START_CHANGE_TYPE:
        retval = lbvToolStartChange();
	break;

    case LBV_TOOL_LOAD_TYPE:
	retval = lbvToolLoad();
	break;

    case LBV_TOOL_UNLOAD_TYPE:
	retval = lbvToolUnload();
	break;

    case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
	load_tool_table_msg = (LBV_TOOL_LOAD_TOOL_TABLE *) cmd;
	retval = lbvToolLoadToolTable(load_tool_table_msg->file);
	break;

    case LBV_TOOL_SET_OFFSET_TYPE:
	lbv_tool_set_offset_msg = (LBV_TOOL_SET_OFFSET *) cmd;
	retval = lbvToolSetOffset(lbv_tool_set_offset_msg->pocket,
                                  lbv_tool_set_offset_msg->toolno,
                                  lbv_tool_set_offset_msg->offset,
                                  lbv_tool_set_offset_msg->diameter,
                                  lbv_tool_set_offset_msg->frontangle,
                                  lbv_tool_set_offset_msg->backangle,
                                  lbv_tool_set_offset_msg->orientation);
	break;

    case LBV_TOOL_SET_NUMBER_TYPE:
	lbv_tool_set_number_msg = (LBV_TOOL_SET_NUMBER *) cmd;
	retval = lbvToolSetNumber(lbv_tool_set_number_msg->tool);
	break;

	// task commands

    case LBV_TASK_INIT_TYPE:
	retval = lbvTaskInit();
	break;

    case LBV_TASK_ABORT_TYPE:
	// abort everything
	lbvTaskAbort();
        lbvIoAbort(LBV_ABORT_TASK_ABORT);
    for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);
	mdi_execute_abort();
	lbvAbortCleanup(LBV_ABORT_TASK_ABORT);
	retval = 0;
	break;

	// mode and state commands

    case LBV_TASK_SET_MODE_TYPE:
	mode_msg = (LBV_TASK_SET_MODE *) cmd;
	if (lbvStatus->task.mode == LBV_TASK_MODE_AUTO &&
	    lbvStatus->task.interpState != LBV_TASK_INTERP_IDLE &&
	    mode_msg->mode != LBV_TASK_MODE_AUTO) {
	    lbvOperatorError(0, _("Can't switch mode while mode is AUTO and interpreter is not IDLE"));
	} else { // we can honour the modeswitch
	    if (mode_msg->mode == LBV_TASK_MODE_MANUAL &&
		lbvStatus->task.mode != LBV_TASK_MODE_MANUAL) {
		// leaving auto or mdi mode for manual

		/*! \todo FIXME-- duplicate code for abort,
	        also near end of main, when aborting on subordinate errors,
	        and in lbvTaskExecute() */

		// abort motion
		lbvTaskAbort();
		mdi_execute_abort();

		// without lbvTaskPlanClose(), a new run command resumes at
		// aborted line-- feature that may be considered later
		{
		    int was_open = taskplanopen;
		    lbvTaskPlanClose();
		    if (lbv_debug & LBV_DEBUG_INTERP && was_open) {
			rcs_print("lbvTaskPlanClose() called at %s:%d\n",
			      __FILE__, __LINE__);
		    }
		}

		// clear out the pending command
		lbvTaskCommand = 0;
		interp_list.clear();
                lbvStatus->task.currentLine = 0;

		// clear out the interpreter state
		lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
		lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
		stepping = 0;
		steppingWait = 0;

		// now queue up command to resynch interpreter
		lbvTaskQueueCommand(&taskPlanSynchCmd);
	    }
	    retval = lbvTaskSetMode(mode_msg->mode);
	}
	break;

    case LBV_TASK_SET_STATE_TYPE:
	state_msg = (LBV_TASK_SET_STATE *) cmd;
	retval = lbvTaskSetState(state_msg->state);
	break;

	// interpreter commands

    case LBV_TASK_PLAN_CLOSE_TYPE:
        retval = lbvTaskPlanClose();
	if (retval > INTERP_MIN_ERROR) {
	    lbvOperatorError(0, _("failed to close file"));
	    retval = -1;
	} else {
	    retval = 0;
        }
        break;

    case LBV_TASK_PLAN_OPEN_TYPE:
	open_msg = (LBV_TASK_PLAN_OPEN *) cmd;
	retval = lbvTaskPlanOpen(open_msg->file);
	if (retval > INTERP_MIN_ERROR) {
	    retval = -1;
	}
	if (-1 == retval) {
	    lbvOperatorError(0, _("can't open %s"), open_msg->file);
	} else {
	    strcpy(lbvStatus->task.file, open_msg->file);
	    retval = 0;
	}
	break;

    case LBV_TASK_PLAN_EXECUTE_TYPE:
	stepping = 0;
	steppingWait = 0;
	execute_msg = (LBV_TASK_PLAN_EXECUTE *) cmd;
        if (!all_homed() && !no_force_homing) { //!no_force_homing = force homing before MDI
            lbvOperatorError(0, _("Can't issue MDI command when not homed"));
            retval = -1;
            break;
        }
        if (lbvStatus->task.mode != LBV_TASK_MODE_MDI) {
            lbvOperatorError(0, _("Must be in MDI mode to issue MDI command"));
            retval = -1;
            break;
        }
	// track interpState also during MDI - it might be an oword sub call
	lbvStatus->task.interpState = LBV_TASK_INTERP_READING;

	if (execute_msg->command[0] != 0) {
	    char * command = execute_msg->command;
	    if (command[0] == (char) 0xff) {
		// Empty command received. Consider it is NULL
		command = NULL;
	    } else {
		// record initial MDI command
		strcpy(lbvStatus->task.command, execute_msg->command);
	    }

	    int level = lbvTaskPlanLevel();
	    if (lbvStatus->task.mode == LBV_TASK_MODE_MDI) {
		if (mdi_execute_level < 0)
		    mdi_execute_level = level;
	    }

	    execRetval = lbvTaskPlanExecute(command, 0);

	    level = lbvTaskPlanLevel();

	    if (lbvStatus->task.mode == LBV_TASK_MODE_MDI) {
		if (mdi_execute_level == level) {
		    mdi_execute_level = -1;
		} else if (level > 0) {
		    // Still insude call. Need another execute(0) call
		    // but only if we didnt encounter an error
		    if (execRetval == INTERP_ERROR) {
			mdi_execute_next = 0;
		    } else {
			mdi_execute_next = 1;
		    }
		}
	    }
	    switch (execRetval) {

	    case INTERP_EXECUTE_FINISH:
		// Flag MDI wait
		mdi_execute_wait = 1;
		// need to flush execution, so signify no more reading
		// until all is done
		lbvTaskPlanSetWait();
		// and resynch the interpreter WM
		lbvTaskQueueCommand(&taskPlanSynchCmd);
		// it's success, so retval really is 0
		retval = 0;
		break;

	    case INTERP_ERROR:
		// lbvStatus->task.interpState =  LBV_TASK_INTERP_WAITING;
		interp_list.clear();
		// abort everything
		lbvTaskAbort();
		lbvIoAbort(LBV_ABORT_INTERPRETER_ERROR_MDI);
	    for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);
		mdi_execute_abort(); // sets lbvStatus->task.interpState to  LBV_TASK_INTERP_IDLE
		lbvAbortCleanup(LBV_ABORT_INTERPRETER_ERROR_MDI, "interpreter error during MDI");
		retval = -1;
		break;

	    case INTERP_EXIT:
	    case INTERP_ENDFILE:
	    case INTERP_FILE_NOT_OPEN:
		// this caused the error msg on M2 in MDI mode - execRetval == INTERP_EXIT which is would be ok (I think). mah
		retval = -1;
		break;

	    default:
		// other codes are OK
		retval = 0;
	    }
	}
	break;

    case LBV_TASK_PLAN_RUN_TYPE:
        if (!all_homed() && !no_force_homing) { //!no_force_homing = force homing before Auto
            lbvOperatorError(0, _("Can't run a program when not homed"));
            retval = -1;
            break;
        }
	stepping = 0;
	steppingWait = 0;
	if (!taskplanopen && lbvStatus->task.file[0] != 0) {
	    lbvTaskPlanOpen(lbvStatus->task.file);
	}
	run_msg = (LBV_TASK_PLAN_RUN *) cmd;
	programStartLine = run_msg->line;
	lbvStatus->task.interpState = LBV_TASK_INTERP_READING;
	lbvStatus->task.task_paused = 0;
	retval = 0;
	break;

    case LBV_TASK_PLAN_PAUSE_TYPE:
	lbvTrajPause();
	if (lbvStatus->task.interpState != LBV_TASK_INTERP_PAUSED) {
	    interpResumeState = lbvStatus->task.interpState;
	}
	lbvStatus->task.interpState = LBV_TASK_INTERP_PAUSED;
	lbvStatus->task.task_paused = 1;
	retval = 0;
	break;

    case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
	if (GET_OPTIONAL_PROGRAM_STOP() == ON) {
	    lbvTrajPause();
	    if (lbvStatus->task.interpState != LBV_TASK_INTERP_PAUSED) {
		interpResumeState = lbvStatus->task.interpState;
	    }
	    lbvStatus->task.interpState = LBV_TASK_INTERP_PAUSED;
	    lbvStatus->task.task_paused = 1;
	}
	retval = 0;
	break;

    case LBV_TASK_PLAN_REVERSE_TYPE:
	lbvTrajReverse();
	retval = 0;
	break;

    case LBV_TASK_PLAN_FORWARD_TYPE:
	lbvTrajForward();
	retval = 0;
	break;

    case LBV_TASK_PLAN_RESUME_TYPE:
	lbvTrajResume();
	lbvStatus->task.interpState =
	    (enum LBV_TASK_INTERP_ENUM) interpResumeState;
	lbvStatus->task.task_paused = 0;
	stepping = 0;
	steppingWait = 0;
	retval = 0;
	break;

    case LBV_TASK_PLAN_END_TYPE:
	retval = 0;
	break;

    case LBV_TASK_PLAN_INIT_TYPE:
	retval = lbvTaskPlanInit();
	if (retval > INTERP_MIN_ERROR) {
	    retval = -1;
	}
	break;

    case LBV_TASK_PLAN_SYNCH_TYPE:
	retval = lbvTaskPlanSynch();
	if (retval > INTERP_MIN_ERROR) {
	    retval = -1;
	}
	break;

    case LBV_TASK_PLAN_SET_OPTIONAL_STOP_TYPE:
	os_msg = (LBV_TASK_PLAN_SET_OPTIONAL_STOP *) cmd;
	lbvTaskPlanSetOptionalStop(os_msg->state);
	retval = 0;
	break;

    case LBV_TASK_PLAN_SET_BLOCK_DELETE_TYPE:
	bd_msg = (LBV_TASK_PLAN_SET_BLOCK_DELETE *) cmd;
	lbvTaskPlanSetBlockDelete(bd_msg->state);
	retval = 0;
	break;

    case LBV_EXEC_PLUGIN_CALL_TYPE:
	retval =  lbvPluginCall( (LBV_EXEC_PLUGIN_CALL *) cmd);
	break;

    case LBV_IO_PLUGIN_CALL_TYPE:
	retval =  lbvIoPluginCall( (LBV_IO_PLUGIN_CALL *) cmd);
	break;

     default:
	// unrecognized command
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print_error("ignoring issue of unknown command %d:%s\n",
			    (int)cmd->type, lbv_symbol_lookup(cmd->type));
	}
	retval = 0;		// don't consider this an error
	break;
    }

    if (retval == -1) {
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print_error("error executing command %d:%s\n", (int)cmd->type,
			    lbv_symbol_lookup(cmd->type));
	}
    }
    /* debug */
    if ((lbv_debug & LBV_DEBUG_TASK_ISSUE) && retval) {
    	rcs_print("lbvTaskIssueCommand() returning: %d\n", retval);
    }
    return retval;
}

/*
   lbvTaskCheckPostconditions() is called for commands on the interp_list.
   Immediate commands, i.e., commands sent from calls to lbvTaskIssueCommand()
   in lbvTaskPlan() directly, are not handled here.

   The return value is a state for lbvTaskExecute() to wait on, e.g.,
   LBV_TASK_EXEC_WAITING_FOR_MOTION, after the command has finished and
   before any other commands can be sent out.
   */
static int lbvTaskCheckPostconditions(NMLmsg * cmd)
{
    if (0 == cmd) {
	return LBV_TASK_EXEC_DONE;
    }

    switch (cmd->type) {
    case LBV_OPERATOR_ERROR_TYPE:
    case LBV_OPERATOR_TEXT_TYPE:
    case LBV_OPERATOR_DISPLAY_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_SYSTEM_CMD_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_SYSTEM_CMD;
	break;

    case LBV_TRAJ_LINEAR_MOVE_TYPE:
    case LBV_TRAJ_CIRCULAR_MOVE_TYPE:
    case LBV_TRAJ_SET_VELOCITY_TYPE:
    case LBV_TRAJ_SET_ACCELERATION_TYPE:
    case LBV_TRAJ_SET_TERM_COND_TYPE:
    case LBV_TRAJ_SET_SPINDLESYNC_TYPE:
    case LBV_TRAJ_SET_OFFSET_TYPE:
    case LBV_TRAJ_SET_G5X_TYPE:
    case LBV_TRAJ_SET_G92_TYPE:
    case LBV_TRAJ_SET_ROTATION_TYPE:
    case LBV_TRAJ_PROBE_TYPE:
    case LBV_TRAJ_RIGID_TAP_TYPE:
    case LBV_TRAJ_CLEAR_PROBE_TRIPPED_FLAG_TYPE:
    case LBV_TRAJ_SET_TELEOP_ENABLE_TYPE:
    case LBV_TRAJ_SET_FO_ENABLE_TYPE:
    case LBV_TRAJ_SET_FH_ENABLE_TYPE:
    case LBV_TRAJ_SET_SO_ENABLE_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_TOOL_PREPARE_TYPE:
    case LBV_TOOL_LOAD_TYPE:
    case LBV_TOOL_UNLOAD_TYPE:
    case LBV_TOOL_LOAD_TOOL_TABLE_TYPE:
    case LBV_TOOL_START_CHANGE_TYPE:
    case LBV_TOOL_SET_OFFSET_TYPE:
    case LBV_TOOL_SET_NUMBER_TYPE:
    case LBV_SPINDLE_SPEED_TYPE:
    case LBV_SPINDLE_ON_TYPE:
    case LBV_SPINDLE_OFF_TYPE:
    case LBV_SPINDLE_ORIENT_TYPE:
    case LBV_COOLANT_MIST_ON_TYPE:
    case LBV_COOLANT_MIST_OFF_TYPE:
    case LBV_COOLANT_FLOOD_ON_TYPE:
    case LBV_COOLANT_FLOOD_OFF_TYPE:
    case LBV_LUBE_ON_TYPE:
    case LBV_LUBE_OFF_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_TASK_PLAN_RUN_TYPE:
    case LBV_TASK_PLAN_PAUSE_TYPE:
    case LBV_TASK_PLAN_END_TYPE:
    case LBV_TASK_PLAN_INIT_TYPE:
    case LBV_TASK_PLAN_SYNCH_TYPE:
    case LBV_TASK_PLAN_EXECUTE_TYPE:
    case LBV_TASK_PLAN_OPTIONAL_STOP_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_SPINDLE_WAIT_ORIENT_COMPLETE_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_SPINDLE_ORIENTED;
	break;

    case LBV_TRAJ_DELAY_TYPE:
    case LBV_AUX_INPUT_WAIT_TYPE:
	return LBV_TASK_EXEC_WAITING_FOR_DELAY;
	break;

    case LBV_MOTION_SET_AOUT_TYPE:
    case LBV_MOTION_SET_DOUT_TYPE:
    case LBV_MOTION_ADAPTIVE_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;

    case LBV_EXEC_PLUGIN_CALL_TYPE:
    case LBV_IO_PLUGIN_CALL_TYPE:
	return LBV_TASK_EXEC_DONE;
	break;

    default:
	// unrecognized command
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print_error("postconditions: unrecognized command %d:%s\n",
			    (int)cmd->type, lbv_symbol_lookup(cmd->type));
	}
	return LBV_TASK_EXEC_DONE;
	break;
    }
    return LBV_TASK_EXEC_DONE; // unreached
}

/*
  STEPPING_CHECK() is a macro that prefaces a switch-case with a check
  for stepping. If stepping is active, it waits until the step has been
  given, then falls through to the rest of the case statement.
*/

#define STEPPING_CHECK()                                                   \
if (stepping) {                                                            \
  if (! steppingWait) {                                                    \
    steppingWait = 1;                                                      \
    steppedLine = lbvStatus->task.currentLine;                             \
  }                                                                        \
  else {                                                                   \
    if (lbvStatus->task.currentLine != steppedLine) {                      \
      break;                                                               \
    }                                                                      \
  }                                                                        \
}

// executor function
static int lbvTaskExecute(void)
{
    int retval = 0;
    int status;			// status of child from LBV_SYSTEM_CMD
    pid_t pid;			// pid returned from waitpid()

    // first check for an abandoned system command and abort it
    if (lbvSystemCmdPid != 0 &&
	lbvStatus->task.execState !=
	LBV_TASK_EXEC_WAITING_FOR_SYSTEM_CMD) {
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print("lbvSystemCmd: abandoning process %d\n",
		      lbvSystemCmdPid);
	}
	kill(lbvSystemCmdPid, SIGINT);
	lbvSystemCmdPid = 0;
    }

    switch (lbvStatus->task.execState) {
    case LBV_TASK_EXEC_ERROR:

	/*! \todo FIXME-- duplicate code for abort,
	   also near end of main, when aborting on subordinate errors,
	   and in lbvTaskIssueCommand() */

	// abort everything
	lbvTaskAbort();
        lbvIoAbort(LBV_ABORT_TASK_EXEC_ERROR);
    for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);
	mdi_execute_abort();

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

	// clear out pending command
	lbvTaskCommand = 0;
	interp_list.clear();
	lbvAbortCleanup(LBV_ABORT_TASK_EXEC_ERROR);
        lbvStatus->task.currentLine = 0;

	// clear out the interpreter state
	lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
	lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	stepping = 0;
	steppingWait = 0;

	// now queue up command to resynch interpreter
	lbvTaskQueueCommand(&taskPlanSynchCmd);

	retval = -1;
	break;

    case LBV_TASK_EXEC_DONE:
	STEPPING_CHECK();
	if (!lbvStatus->motion.traj.queueFull &&
	    lbvStatus->task.interpState != LBV_TASK_INTERP_PAUSED) {
	    if (0 == lbvTaskCommand) {
		// need a new command
		lbvTaskCommand = interp_list.get();
		// interp_list now has line number associated with this-- get
		// it
		if (0 != lbvTaskCommand) {
		    lbvTaskEager = 1;
		    lbvStatus->task.currentLine =
			interp_list.get_line_number();
		    lbvStatus->task.callLevel = lbvTaskPlanLevel();
		    // and set it for all subsystems which use queued ids
		    lbvTrajSetMotionId(lbvStatus->task.currentLine);
		    if (lbvStatus->motion.traj.queueFull) {
			lbvStatus->task.execState =
			    LBV_TASK_EXEC_WAITING_FOR_MOTION_QUEUE;
		    } else {
			lbvStatus->task.execState =
			    (enum LBV_TASK_EXEC_ENUM)
			    lbvTaskCheckPreconditions(lbvTaskCommand);
		    }
		}
	    } else {
		// have an outstanding command
		if (0 != lbvTaskIssueCommand(lbvTaskCommand)) {
		    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
		    retval = -1;
		} else {
		    lbvStatus->task.execState = (enum LBV_TASK_EXEC_ENUM)
			lbvTaskCheckPostconditions(lbvTaskCommand);
		    lbvTaskEager = 1;
		}
		lbvTaskCommand = 0;	// reset it
	    }
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_MOTION_QUEUE:
	STEPPING_CHECK();
	if (!lbvStatus->motion.traj.queueFull) {
	    if (0 != lbvTaskCommand) {
		lbvStatus->task.execState = (enum LBV_TASK_EXEC_ENUM)
		    lbvTaskCheckPreconditions(lbvTaskCommand);
		lbvTaskEager = 1;
	    } else {
		lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
		lbvTaskEager = 1;
	    }
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_MOTION:
	STEPPING_CHECK();
	if (lbvStatus->motion.status == RCS_ERROR) {
	    // lbvOperatorError(0, "error in motion controller");
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	} else if (lbvStatus->motion.status == RCS_DONE) {
	    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	    lbvTaskEager = 1;
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_IO:
	STEPPING_CHECK();
	if (lbvStatus->io.status == RCS_ERROR) {
	    // lbvOperatorError(0, "error in IO controller");
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	} else if (lbvStatus->io.status == RCS_DONE) {
	    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	    lbvTaskEager = 1;
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO:
	STEPPING_CHECK();
	if (lbvStatus->motion.status == RCS_ERROR) {
	    // lbvOperatorError(0, "error in motion controller");
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	} else if (lbvStatus->io.status == RCS_ERROR) {
	    // lbvOperatorError(0, "error in IO controller");
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	} else if (lbvStatus->motion.status == RCS_DONE &&
		   lbvStatus->io.status == RCS_DONE) {
	    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	    lbvTaskEager = 1;
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_SPINDLE_ORIENTED:
	STEPPING_CHECK(); // not sure
	{int state = 0;
		for (int n = 0; n < lbvStatus->motion.traj.spindles; n++){
			if (lbvStatus->motion.spindle[n].orient_state > state)
				state = lbvStatus->motion.spindle[n].orient_state;
		}
	switch (state) {
		case LBVMOT_ORIENT_NONE:
		case LBVMOT_ORIENT_COMPLETE:
			lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
			lbvStatus->task.delayLeft = 0;
			lbvTaskEager = 1;
			rcs_print("wait for orient complete: nothing to do\n");
			break;

		case LBVMOT_ORIENT_IN_PROGRESS:
			lbvStatus->task.delayLeft = taskExecDelayTimeout - etime();
			if (etime() >= taskExecDelayTimeout) {
			lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
			lbvStatus->task.delayLeft = 0;
			lbvTaskEager = 1;
			lbvOperatorError(0, "wait for orient complete: TIMED OUT");
			}
			break;

		case LBVMOT_ORIENT_FAULTED:
			// actually the code in main() should trap this before we get here
			lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
			lbvStatus->task.delayLeft = 0;
			lbvTaskEager = 1;
			for (int n = 0; n < lbvStatus->motion.traj.spindles; n++){
				if (lbvStatus->motion.spindle[n].orient_fault)
						lbvOperatorError(0, "wait for orient complete: FAULTED code=%d",
						lbvStatus->motion.spindle[n].orient_fault);
			}
		}
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_DELAY:
	STEPPING_CHECK();
	// check if delay has passed
	lbvStatus->task.delayLeft = taskExecDelayTimeout - etime();
	if (etime() >= taskExecDelayTimeout) {
	    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	    lbvStatus->task.delayLeft = 0;
	    if (lbvStatus->task.input_timeout != 0)
		lbvStatus->task.input_timeout = 1; // timeout occurred
	    lbvTaskEager = 1;
	}
	// delay can be also be because we wait for an input
	// if the index is set (not -1)
	if (lbvAuxInputWaitIndex >= 0) { 
	    switch (lbvAuxInputWaitType) {
		case WAIT_MODE_HIGH:
		    if (lbvStatus->motion.synch_di[lbvAuxInputWaitIndex] != 0) {
			lbvStatus->task.input_timeout = 0; // clear timeout flag
			lbvAuxInputWaitIndex = -1;
			lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
			lbvStatus->task.delayLeft = 0;
		    }
		    break;

    		case WAIT_MODE_RISE: 
		    if (lbvStatus->motion.synch_di[lbvAuxInputWaitIndex] == 0) {
			lbvAuxInputWaitType = WAIT_MODE_HIGH;
		    }
		    break;
		    
		case WAIT_MODE_LOW:
		    if (lbvStatus->motion.synch_di[lbvAuxInputWaitIndex] == 0) {
			lbvStatus->task.input_timeout = 0; // clear timeout flag
			lbvAuxInputWaitIndex = -1;
			lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
			lbvStatus->task.delayLeft = 0;
		    }
		    break;

		case WAIT_MODE_FALL: //FIXME: implement different fall mode if needed
		    if (lbvStatus->motion.synch_di[lbvAuxInputWaitIndex] != 0) {
			lbvAuxInputWaitType = WAIT_MODE_LOW;
		    }
		    break;

		case WAIT_MODE_IMMEDIATE:
		    lbvStatus->task.input_timeout = 0; // clear timeout flag
		    lbvAuxInputWaitIndex = -1;
		    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
		    lbvStatus->task.delayLeft = 0;
		    break;
		
		default:
		    lbvOperatorError(0, "Unknown Wait Mode");
	    }
	}
	break;

    case LBV_TASK_EXEC_WAITING_FOR_SYSTEM_CMD:
	STEPPING_CHECK();

	// if we got here without a system command pending, say we're done
	if (0 == lbvSystemCmdPid) {
	    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	    break;
	}
	// check the status of the system command
	pid = waitpid(lbvSystemCmdPid, &status, WNOHANG);

	if (0 == pid) {
	    // child is still executing
	    break;
	}

	if (-1 == pid) {
	    // execution error
	    if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
		rcs_print("lbvSystemCmd: error waiting for %d\n",
			  lbvSystemCmdPid);
	    }
	    lbvSystemCmdPid = 0;
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	    break;
	}

	if (lbvSystemCmdPid != pid) {
	    // somehow some other child finished, which is a coding error
	    if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
		rcs_print
		    ("lbvSystemCmd: error waiting for system command %d, we got %d\n",
		     lbvSystemCmdPid, pid);
	    }
	    lbvSystemCmdPid = 0;
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	    break;
	}
	// else child has finished
	if (WIFEXITED(status)) {
	    if (0 == WEXITSTATUS(status)) {
		// child exited normally
		lbvSystemCmdPid = 0;
		lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
		lbvTaskEager = 1;
	    } else {
		// child exited with non-zero status
		if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
		    rcs_print
			("lbvSystemCmd: system command %d exited abnormally with value %d\n",
			 lbvSystemCmdPid, WEXITSTATUS(status));
		}
		lbvSystemCmdPid = 0;
		lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	    }
	} else if (WIFSIGNALED(status)) {
	    // child exited with an uncaught signal
	    if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
		rcs_print("system command %d terminated with signal %d\n",
			  lbvSystemCmdPid, WTERMSIG(status));
	    }
	    lbvSystemCmdPid = 0;
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	} else if (WIFSTOPPED(status)) {
	    // child is currently being traced, so keep waiting
	} else {
	    // some other status, we'll call this an error
	    lbvSystemCmdPid = 0;
	    lbvStatus->task.execState = LBV_TASK_EXEC_ERROR;
	}
	break;

    default:
	// coding error
	if (lbv_debug & LBV_DEBUG_TASK_ISSUE) {
	    rcs_print_error("invalid execState");
	}
	retval = -1;
	break;
    }
    return retval;
}

// called to allocate and init resources
static int lbvtask_startup()
{
    double end;
    int good;

#define RETRY_TIME 10.0		// seconds to wait for subsystems to come up
#define RETRY_INTERVAL 1.0	// seconds between wait tries for a subsystem

    // moved up so it can be exposed in taskmodule at init time
    // // get our status data structure
    // lbvStatus = new LBV_STAT;

    // get the NML command buffer
    if (!(lbv_debug & LBV_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (NULL != lbvCommandBuffer) {
	    delete lbvCommandBuffer;
	}
	lbvCommandBuffer =
	    new RCS_CMD_CHANNEL(lbvFormat, "lbvCommand", "lbv",
				lbv_nmlfile);
	if (lbvCommandBuffer->valid()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't get lbvCommand buffer\n");
	return -1;
    }
    // get our command data structure
    lbvCommand = lbvCommandBuffer->get_address();

    // get the NML status buffer
    if (!(lbv_debug & LBV_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (NULL != lbvStatusBuffer) {
	    delete lbvStatusBuffer;
	}
	lbvStatusBuffer =
	    new RCS_STAT_CHANNEL(lbvFormat, "lbvStatus", "lbv",
				 lbv_nmlfile);
	if (lbvStatusBuffer->valid()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't get lbvStatus buffer\n");
	return -1;
    }

    if (!(lbv_debug & LBV_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (NULL != lbvErrorBuffer) {
	    delete lbvErrorBuffer;
	}
	lbvErrorBuffer =
	    new NML(nmlErrorFormat, "lbvError", "lbv", lbv_nmlfile);
	if (lbvErrorBuffer->valid()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't get lbvError buffer\n");
	return -1;
    }
    // get the timer
    if (!lbvTaskNoDelay) {
	timer = new RCS_TIMER(lbv_task_cycle_time, "", "");
    }
    // initialize the subsystems

    // IO first

    if (!(lbv_debug & LBV_DEBUG_NML)) {
	set_rcs_print_destination(RCS_PRINT_TO_NULL);	// inhibit diag
	// messages
    }
    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == lbvIoInit()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
    // messages
    if (!good) {
	rcs_print_error("can't initialize IO\n");
	return -1;
    }

    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == lbvIoUpdate(&lbvStatus->io)) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rcs_print_error("can't read IO status\n");
	return -1;
    }


    // now motion

    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == lbvMotionInit()) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rcs_print_error("can't initialize motion\n");
	return -1;
    }

    if (setup_inihal() != 0) {
	rcs_print_error("%s: failed to setup inihal\n", __FUNCTION__);
	return -1;
    }

    end = RETRY_TIME;
    good = 0;
    do {
	if (0 == lbvMotionUpdate(&lbvStatus->motion)) {
	    good = 1;
	    break;
	}
	esleep(RETRY_INTERVAL);
	end -= RETRY_INTERVAL;
	if (done) {
	    lbvtask_shutdown();
	    exit(1);
	}
    } while (end > 0.0);
    if (!good) {
	rcs_print_error("can't read motion status\n");
	return -1;
    }
    // now the interpreter

    if (0 != lbvTaskPlanInit()) {
	rcs_print_error("can't initialize interpreter\n");
	return -1;
    }

    if (done ) {
	lbvtask_shutdown();
	exit(1);
    }

    // now task
    if (0 != lbvTaskInit()) {
	rcs_print_error("can't initialize task\n");
	return -1;
    }
    lbvTaskUpdate(&lbvStatus->task);

    return 0;
}

// called to deallocate resources
static int lbvtask_shutdown(void)
{
    // shut down the subsystems
    if (0 != lbvStatus) {
	lbvTaskHalt();
	lbvTaskPlanExit();
	lbvMotionHalt();
	lbvIoHalt();
    }
    // delete the timer
    if (0 != timer) {
	delete timer;
	timer = 0;
    }
    // delete the NML channels

    if (0 != lbvErrorBuffer) {
	delete lbvErrorBuffer;
	lbvErrorBuffer = 0;
    }

    if (0 != lbvStatusBuffer) {
	delete lbvStatusBuffer;
	lbvStatusBuffer = 0;
	lbvStatus = 0;
    }

    if (0 != lbvCommandBuffer) {
	delete lbvCommandBuffer;
	lbvCommandBuffer = 0;
	lbvCommand = 0;
    }

    if (0 != lbvStatus) {
	delete lbvStatus;
	lbvStatus = 0;
    }
    return 0;
}

static int iniLoad(const char *filename)
{
    IniFile inifile;
    const char *inistring;
    char version[LINELEN], machine[LINELEN];
    double saveDouble;
    int saveInt;

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
    if (lbv_debug & LBV_DEBUG_RCS) {
	// set_rcs_print_flag(PRINT_EVERYTHING);
	max_rcs_errors_to_print = -1;
    }

    if (lbv_debug & LBV_DEBUG_VERSIONS) {
	if (NULL != (inistring = inifile.Find("VERSION", "LBV"))) {
	    if(sscanf(inistring, "$Revision: %s", version) != 1) {
		strncpy(version, "unknown", LINELEN-1);
	    }
	} else {
	    strncpy(version, "unknown", LINELEN-1);
	}

	if (NULL != (inistring = inifile.Find("MACHINE", "LBV"))) {
	    strncpy(machine, inistring, LINELEN-1);
	} else {
	    strncpy(machine, "unknown", LINELEN-1);
	}
	rcs_print("task: machine: '%s'  version '%s'\n", machine, version);
    }

    if (NULL != (inistring = inifile.Find("NML_FILE", "LBV"))) {
	// copy to global
	strcpy(lbv_nmlfile, inistring);
    } else {
	// not found, use default
    }

    saveInt = lbv_task_interp_max_len; //remember default or previously set value
    if (NULL != (inistring = inifile.Find("INTERP_MAX_LEN", "TASK"))) {
	if (1 == sscanf(inistring, "%d", &lbv_task_interp_max_len)) {
	    if (lbv_task_interp_max_len <= 0) {
	    	lbv_task_interp_max_len = saveInt;
	    }
	} else {
	    lbv_task_interp_max_len = saveInt;
	}
    }

    if (NULL != (inistring = inifile.Find("RS274NGC_STARTUP_CODE", "RS274NGC"))) {
	// copy to global
	strcpy(rs274ngc_startup_code, inistring);
    } else {
	//FIXME-AJ: this is the old (unpreferred) location. just for compatibility purposes
	//it will be dropped in v2.4
	if (NULL != (inistring = inifile.Find("RS274NGC_STARTUP_CODE", "LBV"))) {
	    // copy to global
	    strcpy(rs274ngc_startup_code, inistring);
	} else {
	// not found, use default
	}
    }
    saveDouble = lbv_task_cycle_time;
    LBV_TASK_CYCLE_TIME_ORIG = lbv_task_cycle_time;
    lbvTaskNoDelay = 0;
    if (NULL != (inistring = inifile.Find("CYCLE_TIME", "TASK"))) {
	if (1 == sscanf(inistring, "%lf", &lbv_task_cycle_time)) {
	    // found it
	    // if it's <= 0.0, then flag that we don't want to
	    // wait at all, which will set the LBV_TASK_CYCLE_TIME
	    // global to the actual time deltas
	    if (lbv_task_cycle_time <= 0.0) {
		lbvTaskNoDelay = 1;
	    }
	} else {
	    // found, but invalid
	    lbv_task_cycle_time = saveDouble;
	    rcs_print
		("invalid [TASK] CYCLE_TIME in %s (%s); using default %f\n",
		 filename, inistring, lbv_task_cycle_time);
	}
    } else {
	// not found, using default
	rcs_print("[TASK] CYCLE_TIME not found in %s; using default %f\n",
		  filename, lbv_task_cycle_time);
    }


    if (NULL != (inistring = inifile.Find("NO_FORCE_HOMING", "TRAJ"))) {
	if (1 == sscanf(inistring, "%d", &no_force_homing)) {
	    // found it
	    // if it's <= 0.0, then set it 0 so that homing is required before MDI or Auto
	    if (no_force_homing <= 0) {
		no_force_homing = 0;
	    }
	} else {
	    // found, but invalid
	    no_force_homing = 0;
	    rcs_print
		("invalid [TRAJ] NO_FORCE_HOMING in %s (%s); using default %d\n",
		 filename, inistring, no_force_homing);
	}
    } else {
	// not found, using default
	no_force_homing = 0;
    }

    // configurable template for iocontrol reason display
    if (NULL != (inistring = inifile.Find("IO_ERROR", "TASK"))) {
	io_error = strdup(inistring);
    }

    // max number of queued MDI commands
    if (NULL != (inistring = inifile.Find("MDI_QUEUED_COMMANDS", "TASK"))) {
	max_mdi_queued_commands = atoi(inistring);
    }

    // close it
    inifile.Close();

    return 0;
}

/*
  syntax: a.out {-d -ini <inifile>} {-nml <nmlfile>} {-shm <key>}
  */
int main(int argc, char *argv[])
{
    int taskPlanError = 0;
    int taskExecuteError = 0;
    double startTime, endTime, deltaTime;
    double first_start_time;
    int num_latency_warnings = 0;
    int latency_excursion_factor = 10;  // if latency is worse than (factor * expected), it's an excursion
    double minTime, maxTime;

    bindtextdomain("labvcnc", LBV2_PO_DIR);
    setlocale(LC_MESSAGES,"");
    setlocale(LC_CTYPE,"");
    textdomain("labvcnc");

    // loop until done
    done = 0;
    // trap ^C
    signal(SIGINT, lbvtask_quit);
    // and SIGTERM (used by runscript to shut down)
    signal(SIGTERM, lbvtask_quit);

    // create a backtrace on stderr
    signal(SIGSEGV, backtrace);
    signal(SIGFPE, backtrace);
    signal(SIGUSR1, backtrace);

    // set print destination to stdout, for console apps
    set_rcs_print_destination(RCS_PRINT_TO_STDOUT);
    // process command line args
    if (0 != lbvGetArgs(argc, argv)) {
	rcs_print_error("error in argument list\n");
	exit(1);
    }

    if (done) {
	lbvtask_shutdown();
	exit(1);
    }

    if (done) {
	lbvtask_shutdown();
	exit(1);
    }
    // get configuration information
    iniLoad(lbv_inifile);

    if (done) {
	lbvtask_shutdown();
	exit(1);
    }

    // get our status data structure
    // moved up from lbv_startup so we can expose it in Python right away
    lbvStatus = new LBV_STAT;

    // get the Python plugin going

    // inistantiate task methods object, too
    lbvTaskOnce(lbv_inifile);
    if (task_methods == NULL) {
	set_rcs_print_destination(RCS_PRINT_TO_STDOUT);	// restore diag
	rcs_print_error("can't initialize Task methods\n");
	lbvtask_shutdown();
	exit(1);
    }

    // this is the place to run any post-HAL-creation halcmd files
    lbvRunHalFiles(lbv_inifile);

    // initialize everything
    if (0 != lbvtask_startup()) {
	lbvtask_shutdown();
	exit(1);
    }
    // set the default startup modes
    lbvMotionAbort();
    for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);
    lbvAuxEstopOn();
    for (int t = 0; t < lbvStatus->motion.traj.joints; t++) {
        lbvJointDisable(t);
    }
    lbvTrajDisable();
    lbvLubeOff();
    lbvIoAbort(LBV_ABORT_TASK_STATE_ESTOP);
    lbvJointUnhome(-2);

    lbvTrajSetMode(LBV_TRAJ_MODE_FREE);

    // reflect the initial value of LBV_DEBUG in lbvStatus->debug
    lbvStatus->debug = lbv_debug;

    startTime = etime();	// set start time before entering loop;
    first_start_time = startTime;
    endTime = startTime;
    // it will be set at end of loop from now on
    minTime = DBL_MAX;		// set to value that can never be exceeded
    maxTime = 0.0;		// set to value that can never be underset

    if (0 != usrmotReadLbvmotConfig(&lbvmotConfig)) {
        rcs_print("%s failed usrmotReadLbvmotconfig()\n",__FILE__);
    }
    while (!done) {
        static int gave_soft_limit_message = 0;
        check_ini_hal_items(lbvStatus->motion.traj.joints);
	// read command
	if (0 != lbvCommandBuffer->read()) {
	    // got a new command, so clear out errors
	    taskPlanError = 0;
	    taskExecuteError = 0;
	}
	// run control cycle
	if (0 != lbvTaskPlan()) {
	    taskPlanError = 1;
	}
	if (0 != lbvTaskExecute()) {
	    taskExecuteError = 1;
	}
	// update subordinate status

	lbvIoUpdate(&lbvStatus->io);
	lbvMotionUpdate(&lbvStatus->motion);
	// synchronize subordinate states
	if (lbvStatus->io.aux.estop) {
	    if (lbvStatus->motion.traj.enabled) {
		lbvTrajDisable();
		lbvTaskAbort();
        lbvIoAbort(LBV_ABORT_AUX_ESTOP);
        for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);
        lbvJointUnhome(-2); // only those joints which are volatile_home
		mdi_execute_abort();
		lbvAbortCleanup(LBV_ABORT_AUX_ESTOP);
		lbvTaskPlanSynch();
	    }
	    if (lbvStatus->io.coolant.mist) {
		lbvCoolantMistOff();
	    }
	    if (lbvStatus->io.coolant.flood) {
		lbvCoolantFloodOff();
	    }
	    if (lbvStatus->io.lube.on) {
		lbvLubeOff();
	    }
	    for (int n = 0; n < lbvStatus->motion.traj.spindles; n++){
	    	if (lbvStatus->motion.spindle[n].enabled) {
	    		lbvSpindleOff(n);
	    	}
	    }
	}

	// toolchanger indicated fault code > 0
	if ((lbvStatus->io.status == RCS_ERROR) &&
	    lbvStatus->io.fault) {
	    static int reported = -1;
	    if (lbvStatus->io.reason > 0) {
		if (reported ^ lbvStatus->io.fault) {
		    rcs_print("M6: toolchanger soft fault=%d, reason=%d\n",
			      lbvStatus->io.fault, lbvStatus->io.reason);
		    reported = lbvStatus->io.fault;
		}
		lbvStatus->io.status = RCS_DONE; // let program continue
	    } else {
		rcs_print("M6: toolchanger hard fault, reason=%d\n",
			  lbvStatus->io.reason);
		// abort since io.status is RCS_ERROR
	    }

	}

        if (!lbvStatus->motion.on_soft_limit) {gave_soft_limit_message = 0;}

	// check for subordinate errors, and halt task if so
        if (   lbvStatus->motion.status == RCS_ERROR
            && lbvStatus->motion.on_soft_limit) { 
           if (!gave_soft_limit_message) {
                lbvOperatorError(0, "On Soft Limit");
                // if gui does not provide a means to switch to joint mode
                // the  machine may be stuck (a misconfiguration)
                if (lbvmotConfig.kinType == KINEMATICS_IDENTITY) {
                    lbvOperatorError(0,"Identity kinematics are MISCONFIGURED");
                }
                gave_soft_limit_message = 1;
           }
        } else if (lbvStatus->motion.status == RCS_ERROR ||
	    ((lbvStatus->io.status == RCS_ERROR) &&
	     (lbvStatus->io.reason <= 0))) {
	    /*! \todo FIXME-- duplicate code for abort,
	      also in lbvTaskExecute()
	      and in lbvTaskIssueCommand() */

	    if (lbvStatus->io.status == RCS_ERROR) {
		// this is an aborted M6.
		if (lbv_debug & LBV_DEBUG_RCS ) {
		    rcs_print("io.status=RCS_ERROR, fault=%d reason=%d\n",
			      lbvStatus->io.fault, lbvStatus->io.reason);
		}
		if (lbvStatus->io.reason < 0) {
		    lbvOperatorError(0, io_error, lbvStatus->io.reason);
		}
	    }
	    // motion already should have reported this condition (and set RCS_ERROR?)
	    // an M19 orient failed to complete within timeout
	    // if ((lbvStatus->motion.status == RCS_ERROR) && 
	    // 	(lbvStatus->motion.spindle.orient_state == LBVMOT_ORIENT_FAULTED) &&
	    // 	(lbvStatus->motion.spindle.orient_fault != 0)) {
	    // 	lbvOperatorError(0, "wait for orient complete timed out");
	    // }

            // abort everything
            lbvTaskAbort();
            lbvIoAbort(LBV_ABORT_MOTION_OR_IO_RCS_ERROR);
        for (int s = 0; s < lbvStatus->motion.traj.spindles; s++) lbvSpindleAbort(s);;
	    mdi_execute_abort();
	    // without lbvTaskPlanClose(), a new run command resumes at
	    // aborted line-- feature that may be considered later
	    {
		int was_open = taskplanopen;
		lbvTaskPlanClose();
		if (lbv_debug & LBV_DEBUG_INTERP && was_open) {
		    rcs_print("lbvTaskPlanClose() called at %s:%d\n",
			      __FILE__, __LINE__);
		}
	    }

	    // clear out the pending command
	    lbvTaskCommand = 0;
	    interp_list.clear();
	    lbvStatus->task.currentLine = 0;

	    lbvAbortCleanup(LBV_ABORT_MOTION_OR_IO_RCS_ERROR);

	    // clear out the interpreter state
	    lbvStatus->task.interpState = LBV_TASK_INTERP_IDLE;
	    lbvStatus->task.execState = LBV_TASK_EXEC_DONE;
	    stepping = 0;
	    steppingWait = 0;

	    // now queue up command to resynch interpreter
	    lbvTaskQueueCommand(&taskPlanSynchCmd);
	}

	// update task-specific status
	lbvTaskUpdate(&lbvStatus->task);

	// handle RCS_STAT_MSG base class members explicitly, since this
	// is not an NML_MODULE and they won't be set automatically

	// do task
	lbvStatus->task.command_type = lbvCommand->type;
	lbvStatus->task.echo_serial_number = lbvCommand->serial_number;

	// do top level
	lbvStatus->command_type = lbvCommand->type;
	lbvStatus->echo_serial_number = lbvCommand->serial_number;

	if (taskPlanError || taskExecuteError ||
	    lbvStatus->task.execState == LBV_TASK_EXEC_ERROR ||
	    lbvStatus->motion.status == RCS_ERROR ||
	    lbvStatus->io.status == RCS_ERROR) {
	    lbvStatus->status = RCS_ERROR;
	    lbvStatus->task.status = RCS_ERROR;
	} else if (!taskPlanError && !taskExecuteError &&
		   lbvStatus->task.execState == LBV_TASK_EXEC_DONE &&
		   lbvStatus->motion.status == RCS_DONE &&
		   lbvStatus->io.status == RCS_DONE &&
		   mdi_execute_queue.len() == 0 &&
		   interp_list.len() == 0 &&
		   lbvTaskCommand == 0 &&
		   lbvStatus->task.interpState == LBV_TASK_INTERP_IDLE) {
	    lbvStatus->status = RCS_DONE;
	    lbvStatus->task.status = RCS_DONE;
	} else {
	    lbvStatus->status = RCS_EXEC;
	    lbvStatus->task.status = RCS_EXEC;
	}

	// write it
	// since lbvStatus was passed to the WM init functions, it
	// will be updated in the _update() functions above. There's
	// no need to call the individual functions on all WM items.
	lbvStatusBuffer->write(lbvStatus);

	// wait on timer cycle, if specified, or calculate actual
	// interval if ini file says to run full out via
	// [TASK] CYCLE_TIME <= 0.0d
	// lbvTaskEager = 0;
        endTime = etime();
        deltaTime = endTime - startTime;
        if (deltaTime < minTime)
            minTime = deltaTime;
        else if (deltaTime > maxTime)
            maxTime = deltaTime;
        startTime = endTime;
        if (deltaTime > (latency_excursion_factor * lbv_task_cycle_time)) {
            if (num_latency_warnings < 10) {
                rcs_print("task: main loop took %.6f seconds\n", deltaTime);
            }
            num_latency_warnings ++;
        }

	if ((lbvTaskNoDelay) || (lbvTaskEager)) {
	    lbvTaskEager = 0;
	} else {
	    timer->wait();
	}
    }
    // end of while (! done)

    rcs_print(
        "task: %u cycles, min=%.6f, max=%.6f, avg=%.6f, %u latency excursions (> %dx expected cycle time of %.6fs)\n",
        lbvStatus->task.heartbeat,
        minTime,
        maxTime,
        (lbvStatus->task.heartbeat != 0) ?  (endTime - first_start_time) / lbvStatus->task.heartbeat : -1.0,
        num_latency_warnings,
        latency_excursion_factor,
        lbv_task_cycle_time
    );

    // clean up everything
    lbvtask_shutdown();

    // and leave
    exit(0);
}
