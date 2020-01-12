
//
// motion-logger: a test program to log motion commands sent from
//     LabvCNC's Task module to the Motion module
//
// Copyright (C) 2015, Sebastian Kuzminsky <seb@highlab.com>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//

#include <errno.h>
#include <float.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "hal.h"
#include "motion_debug.h"
#include "motion.h"
#include "motion_struct.h"
#include "motion_types.h"
#include "mot_priv.h"

static struct motion_logger_data_t {
    hal_bit_t *reopen;
} *motion_logger_data;

FILE *logfile = NULL;
char *logfile_name = NULL;

lbvmot_struct_t *lbvmotStruct = 0;

struct lbvmot_command_t *c = 0;
struct lbvmot_status_t *lbvmotStatus = 0;
struct lbvmot_config_t *lbvmotConfig = 0;
struct lbvmot_debug_t *lbvmotDebug = 0;
struct lbvmot_internal_t *lbvmotInternal = 0;
struct lbvmot_error_t *lbvmotError = 0;

int mot_comp_id;

lbvmot_joint_t joint_array[LBVMOT_MAX_JOINTS];
int num_joints = LBVMOT_MAX_JOINTS;
lbvmot_joint_t *joints = 0;
int num_spindles = LBVMOT_MAX_SPINDLES;

lbvmot_axis_t axis_array[LBVMOT_MAX_AXIS];
int num_axes = LBVMOT_MAX_AXIS;
lbvmot_axis_t *axes = 0;

void lbvmot_config_change(void) {
    if (lbvmotConfig->head == lbvmotConfig->tail) {
        lbvmotConfig->head++;
        lbvmotConfig->config_num++;
        lbvmotStatus->config_num = lbvmotConfig->config_num;
        lbvmotConfig->tail = lbvmotConfig->head;
    }
}


static int init_comm_buffers(void) {
    int joint_num, axis_num, n;
    lbvmot_joint_t *joint;
    lbvmot_axis_t *axis;
    int retval;
    int shmem_id;

    rtapi_print_msg(RTAPI_MSG_INFO,
	"MOTION: init_comm_buffers() starting...\n");

    lbvmotStruct = 0;
    lbvmotDebug = 0;
    lbvmotStatus = 0;
    c = 0;
    lbvmotConfig = 0;

    /* allocate and initialize the shared memory structure */
    shmem_id = rtapi_shmem_new(DEFAULT_SHMEM_KEY, mot_comp_id, sizeof(lbvmot_struct_t));
    if (shmem_id < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_new failed, returned %d\n", shmem_id);
	return -1;
    }
    retval = rtapi_shmem_getptr(shmem_id, (void **) &lbvmotStruct);
    if (retval < 0) {
	rtapi_print_msg(RTAPI_MSG_ERR,
	    "MOTION: rtapi_shmem_getptr failed, returned %d\n", retval);
	return -1;
    }

    /* zero shared memory before doing anything else. */
    memset(lbvmotStruct, 0, sizeof(lbvmot_struct_t));

    /* we'll reference lbvmotStruct directly */
    c = &lbvmotStruct->command;
    lbvmotStatus = &lbvmotStruct->status;
    lbvmotConfig = &lbvmotStruct->config;
    lbvmotDebug = &lbvmotStruct->debug;
    lbvmotInternal = &lbvmotStruct->internal;
    lbvmotError = &lbvmotStruct->error;

    lbvmotConfig->numJoints = num_joints;
    lbvmotConfig->numSpindles = num_spindles;

    lbvmotStatus->vel = DEFAULT_VELOCITY;
    lbvmotConfig->limitVel = DEFAULT_VELOCITY;
    lbvmotStatus->acc = DEFAULT_ACCELERATION;
    lbvmotStatus->feed_scale = 1.0;
    lbvmotStatus->rapid_scale = 1.0;
    for (int n = 0; n < LBVMOT_MAX_SPINDLES; n++) lbvmotStatus->spindle_status[n].scale = 1.0;
    lbvmotStatus->net_feed_scale = 1.0;
    /* adaptive feed is off by default, feed override, spindle 
       override, and feed hold are on */
    lbvmotStatus->enables_new = FS_ENABLED | SS_ENABLED | FH_ENABLED;
    lbvmotStatus->enables_queued = lbvmotStatus->enables_new;
    SET_MOTION_INPOS_FLAG(1);
    lbvmotConfig->kinType = KINEMATICS_IDENTITY;

    lbvmot_config_change();

    /* init pointer to joint structs */
    joints = joint_array;

    /* init per-joint stuff */
    for (joint_num = 0; joint_num < num_joints; joint_num++) {
	/* point to structure for this joint */
	joint = &joints[joint_num];

	/* init the config fields with some "reasonable" defaults" */

	joint->max_pos_limit = 1.0;
	joint->min_pos_limit = -1.0;
	joint->vel_limit = 1.0;
	joint->acc_limit = 1.0;
	joint->min_ferror = 0.01;
	joint->max_ferror = 1.0;

	joint->comp.entry = &(joint->comp.array[0]);
	/* the compensation code has -DBL_MAX at one end of the table
	   and +DBL_MAX at the other so _all_ commanded positions are
	   guaranteed to be covered by the table */
	joint->comp.array[0].nominal = -DBL_MAX;
	joint->comp.array[0].fwd_trim = 0.0;
	joint->comp.array[0].rev_trim = 0.0;
	joint->comp.array[0].fwd_slope = 0.0;
	joint->comp.array[0].rev_slope = 0.0;
	for ( n = 1 ; n < LBVMOT_COMP_SIZE+2 ; n++ ) {
	    joint->comp.array[n].nominal = DBL_MAX;
	    joint->comp.array[n].fwd_trim = 0.0;
	    joint->comp.array[n].rev_trim = 0.0;
	    joint->comp.array[n].fwd_slope = 0.0;
	    joint->comp.array[n].rev_slope = 0.0;
	}

	/* init status info */
	joint->ferror_limit = joint->min_ferror;

	/* init misc other stuff in joint structure */
	joint->big_vel = 10.0 * joint->vel_limit;

	SET_JOINT_INPOS_FLAG(joint, 1);
    }

    /* init pointer to axes structs */
    axes = axis_array;

    /* init per-axis stuff */
    for (axis_num = 0; axis_num < num_axes; axis_num++) {
	/* point to structure for this axis */
	axis = &axes[axis_num];
	axis->pos_cmd = 0.0;
	axis->teleop_vel_cmd = 0.0;
	axis->max_pos_limit = 1.0;
	axis->min_pos_limit = -1.0;
	axis->vel_limit = 1.0;
	axis->acc_limit = 1.0;
	//simple_tp_t teleop_tp
	axis->old_ajog_counts = 0;
	axis->kb_ajog_active = 0;
	axis->wheel_ajog_active = 0;
    }

    lbvmotDebug->start_time = time(NULL);

    rtapi_print_msg(RTAPI_MSG_INFO, "MOTION: init_comm_buffers() complete\n");
    return 0;
}


void update_motion_state(void) {
    if (GET_MOTION_ENABLE_FLAG()) {
        if (GET_MOTION_TELEOP_FLAG()) {
            lbvmotStatus->motion_state = LBVMOT_MOTION_TELEOP;
        } else if (GET_MOTION_COORD_FLAG()) {
            lbvmotStatus->motion_state = LBVMOT_MOTION_COORD;
        } else {
            lbvmotStatus->motion_state = LBVMOT_MOTION_FREE;
        }
    } else {
        lbvmotStatus->motion_state = LBVMOT_MOTION_DISABLED;
    }
}


void update_joint_status(void) {
    for (int j = 0; j < num_joints; j ++) {
        lbvmot_joint_status_t *joint_status;
        lbvmot_joint_t *joint;

        joint_status = &lbvmotStatus->joint_status[j];
        joint = &joints[j];

	joint_status->flag = joint->flag;
	joint_status->pos_cmd = joint->pos_cmd;
	joint_status->pos_fb = joint->pos_fb;
	joint_status->vel_cmd = joint->vel_cmd;
	joint_status->acc_cmd = joint->acc_cmd;
	joint_status->ferror = joint->ferror;
	joint_status->ferror_high_mark = joint->ferror_high_mark;
	joint_status->backlash = joint->backlash;
	joint_status->max_pos_limit = joint->max_pos_limit;
	joint_status->min_pos_limit = joint->min_pos_limit;
	joint_status->min_ferror = joint->min_ferror;
	joint_status->max_ferror = joint->max_ferror;
    }
}


static void mark_joint_homed(int joint_num) {
    lbvmot_joint_status_t *joint_status;

    joint_status = &lbvmotStatus->joint_status[joint_num];
    joint_status->homing = 0;
    joint_status->homed  = 1;
    return;
}

void maybe_reopen_logfile() {
    if(*motion_logger_data->reopen) {
        if(logfile != stdout) {
            fclose(logfile);
            logfile = NULL;
        }
        *motion_logger_data->reopen = 0;
    }
}

void log_print(const char *fmt, ...) {
    va_list ap;

    maybe_reopen_logfile();

    if (logfile == NULL) {
        if (logfile_name == NULL) {
            logfile = stdout;
        } else {
            logfile = fopen(logfile_name, "w");
            if (logfile == NULL) {
                fprintf(stderr, "error opening %s: %s\n", logfile_name, strerror(errno));
                exit(1);
            }
        }
    }

    va_start(ap, fmt);
    vfprintf(logfile, fmt, ap);
    va_end(ap);
    fflush(logfile);
}


int main(int argc, char* argv[]) {
    if (argc == 1) {
        logfile = stdout;
        logfile_name = NULL;
    } else if (argc == 2) {
        logfile = NULL;
        logfile_name = argv[1];
    } else {
        fprintf(stderr, "usage: motion-logger [LOGFILE]\n");
        exit(1);
    }

    mot_comp_id = hal_init("motion-logger");
    motion_logger_data = hal_malloc(sizeof(*motion_logger_data));
    int r = hal_pin_bit_new("motion-logger.reopen-log", HAL_IO, &motion_logger_data->reopen,
            mot_comp_id);
    if(r < 0) { errno = -r; perror("hal_pin_bit_new"); exit(1); }
    *motion_logger_data->reopen = 0;
    r = hal_ready(mot_comp_id);
    if(r < 0) { errno = -r; perror("hal_ready"); exit(1); }
    init_comm_buffers();

    while (1) {
        if (c->commandNum != c->tail) {
            // "split read"
            continue;
        }
        if (c->commandNum == lbvmotStatus->commandNumEcho) {
            // nothing new
            maybe_reopen_logfile();
            usleep(10 * 1000);
            continue;
        }

        //
        // new incoming command!
        //

        lbvmotStatus->head++;

        switch (c->command) {
            case LBVMOT_ABORT:
                log_print("ABORT\n");
                break;

            case LBVMOT_JOINT_ABORT:
                log_print("JOINT_ABORT joint=%d\n", c->joint);
                break;

            case LBVMOT_ENABLE:
                log_print("ENABLE\n");
                SET_MOTION_ENABLE_FLAG(1);
                update_motion_state();
                break;

            case LBVMOT_DISABLE:
                log_print("DISABLE\n");
                SET_MOTION_ENABLE_FLAG(0);
                update_motion_state();
                break;

            case LBVMOT_JOINT_ENABLE_AMPLIFIER:
                log_print("ENABLE_AMPLIFIER\n");
                break;

            case LBVMOT_JOINT_DISABLE_AMPLIFIER:
                log_print("DISABLE_AMPLIFIER\n");
                break;

            case LBVMOT_ENABLE_WATCHDOG:
                log_print("ENABLE_WATCHDOG\n");
                break;

            case LBVMOT_DISABLE_WATCHDOG:
                log_print("DISABLE_WATCHDOG\n");
                break;

            case LBVMOT_JOINT_ACTIVATE:
                log_print("JOINT_ACTIVATE joint=%d\n", c->joint);
                break;

            case LBVMOT_JOINT_DEACTIVATE:
                log_print("JOINT_DEACTIVATE joint=%d\n", c->joint);
                break;

            case LBVMOT_PAUSE:
                log_print("PAUSE\n");
                break;

            case LBVMOT_RESUME:
                log_print("RESUME\n");
                break;

            case LBVMOT_STEP:
                log_print("STEP\n");
                break;

            case LBVMOT_FREE:
                log_print("FREE\n");
                SET_MOTION_COORD_FLAG(0);
                SET_MOTION_TELEOP_FLAG(0);
                update_motion_state();
                break;

            case LBVMOT_COORD:
                log_print("COORD\n");
                SET_MOTION_COORD_FLAG(1);
                SET_MOTION_TELEOP_FLAG(0);
                SET_MOTION_ERROR_FLAG(0);
                update_motion_state();
                break;

            case LBVMOT_TELEOP:
                log_print("TELEOP\n");
                SET_MOTION_TELEOP_FLAG(1);
                SET_MOTION_ERROR_FLAG(0);
                update_motion_state();
                break;

            case LBVMOT_SPINDLE_SCALE:
                log_print("SPINDLE_SCALE\n");
                break;

            case LBVMOT_SS_ENABLE:
                log_print("SS_ENABLE\n");
                break;

            case LBVMOT_FEED_SCALE:
                log_print("FEED_SCALE\n");
                break;

            case LBVMOT_RAPID_SCALE:
                log_print("RAPID_SCALE\n");
                break;

            case LBVMOT_FS_ENABLE:
                log_print("FS_ENABLE\n");
                break;

            case LBVMOT_FH_ENABLE:
                log_print("FH_ENABLE\n");
                break;

            case LBVMOT_AF_ENABLE:
                log_print("AF_ENABLE\n");
                break;

            case LBVMOT_OVERRIDE_LIMITS:
                log_print("OVERRIDE_LIMITS\n");
                break;

            case LBVMOT_JOINT_HOME:
                log_print("JOINT_HOME joint=%d\n", c->joint);
                if (c->joint < 0) {
                    for (int j = 0; j < num_joints; j ++) {
                        mark_joint_homed(j);
                    }
                } else {
                    mark_joint_homed(c->joint);
                }
                break;

            case LBVMOT_JOINT_UNHOME:
                log_print("JOINT_UNHOME joint=%d\n", c->joint);
                break;

            case LBVMOT_JOG_CONT:
                log_print("JOG_CONT\n");
                break;

            case LBVMOT_JOG_INCR:
                log_print("JOG_INCR\n");
                break;

            case LBVMOT_JOG_ABS:
                log_print("JOG_ABS\n");
                break;

            case LBVMOT_SET_LINE:
                log_print(
                    "SET_LINE x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f, u=%.6f, v=%.6f, w=%.6f, id=%d, motion_type=%d, vel=%.6f, ini_maxvel=%.6f, acc=%.6f, turn=%d\n",
                    c->pos.tran.x, c->pos.tran.y, c->pos.tran.z,
                    c->pos.a, c->pos.b, c->pos.c,
                    c->pos.u, c->pos.v, c->pos.w,
                    c->id, c->motion_type,
                    c->vel, c->ini_maxvel,
                    c->acc, c->turn
                );
                break;

            case LBVMOT_SET_CIRCLE:
                log_print("SET_CIRCLE:\n");
                log_print(
                    "    pos: x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f, u=%.6f, v=%.6f, w=%.6f\n",
                    c->pos.tran.x, c->pos.tran.y, c->pos.tran.z,
                    c->pos.a, c->pos.b, c->pos.c,
                    c->pos.u, c->pos.v, c->pos.w
                );
                log_print("    center: x=%.6f, y=%.6f, z=%.6f\n", c->center.x, c->center.y, c->center.z);
                log_print("    normal: x=%.6f, y=%.6f, z=%.6f\n", c->normal.x, c->normal.y, c->normal.z);
                log_print("    id=%d, motion_type=%d, vel=%.6f, ini_maxvel=%.6f, acc=%.6f, turn=%d\n",
                    c->id, c->motion_type,
                    c->vel, c->ini_maxvel,
                    c->acc, c->turn
                );
                break;

            case LBVMOT_SET_TELEOP_VECTOR:
                log_print("SET_TELEOP_VECTOR\n");
                break;

            case LBVMOT_CLEAR_PROBE_FLAGS:
                log_print("CLEAR_PROBE_FLAGS\n");
                break;

            case LBVMOT_PROBE:
                log_print("PROBE\n");
                break;

            case LBVMOT_RIGID_TAP:
                log_print("RIGID_TAP\n");
                break;

            case LBVMOT_SET_JOINT_POSITION_LIMITS:
                log_print(
                    "SET_JOINT_POSITION_LIMITS joint=%d, min=%.6f, max=%.6f\n",
                    c->joint, c->minLimit, c->maxLimit
                );
                joints[c->joint].max_pos_limit = c->maxLimit;
                joints[c->joint].min_pos_limit = c->minLimit;
                break;

            case LBVMOT_SET_AXIS_POSITION_LIMITS:
                log_print(
                    "SET_AXIS_POSITION_LIMITS axis=%d, min=%.6f, max=%.6f\n",
                    c->axis, c->minLimit, c->maxLimit
                );
                axes[c->axis].max_pos_limit = c->maxLimit;
                axes[c->axis].min_pos_limit = c->minLimit;
                break;

            case LBVMOT_SET_AXIS_LOCKING_JOINT:
                log_print(
                    "SET_AXIS_LOCKING_JOINT axis=%d, locking_joint=%d\n",
                    c->axis, c->joint
                );
                axes[c->axis].locking_joint = c->joint;
                break;

            case LBVMOT_SET_JOINT_BACKLASH:
                log_print("SET_JOINT_BACKLASH joint=%d, backlash=%.6f\n", c->joint, c->backlash);
                break;

            case LBVMOT_SET_JOINT_MIN_FERROR:
                log_print("SET_JOINT_MIN_FERROR joint=%d, minFerror=%.6f\n", c->joint, c->minFerror);
                break;

            case LBVMOT_SET_JOINT_MAX_FERROR:
                log_print("SET_JOINT_MAX_FERROR joint=%d, maxFerror=%.6f\n", c->joint, c->maxFerror);
                break;

            case LBVMOT_SET_VEL:
                log_print("SET_VEL vel=%.6f, ini_maxvel=%.6f\n", c->vel, c->ini_maxvel);
                break;

            case LBVMOT_SET_VEL_LIMIT:
                log_print("SET_VEL_LIMIT vel=%.6f\n", c->vel);
                break;

            case LBVMOT_SET_AXIS_VEL_LIMIT:
                log_print("SET_AXIS_VEL_LIMIT axis=%d vel=%.6f\n", c->axis, c->vel);
                break;

            case LBVMOT_SET_JOINT_VEL_LIMIT:
                log_print("SET_JOINT_VEL_LIMIT joint=%d, vel=%.6f\n", c->joint, c->vel);
                break;

            case LBVMOT_SET_AXIS_ACC_LIMIT:
                log_print("SET_AXIS_ACC_LIMIT axis=%d, acc=%.6f\n", c->axis, c->acc);
                break;

            case LBVMOT_SET_JOINT_ACC_LIMIT:
                log_print("SET_JOINT_ACC_LIMIT joint=%d, acc=%.6f\n", c->joint, c->acc);
                break;

            case LBVMOT_SET_ACC:
                log_print("SET_ACC acc=%.6f\n", c->acc);
                break;

            case LBVMOT_SET_TERM_COND:
                log_print("SET_TERM_COND termCond=%d, tolerance=%.6f\n", c->termCond, c->tolerance);
                break;

            case LBVMOT_SET_NUM_JOINTS:
                log_print("SET_NUM_JOINTS %d\n", c->joint);
                num_joints = c->joint;
                break;

            case LBVMOT_SET_NUM_SPINDLES:
                log_print("SET_NUM_SPINDLES %d\n", c->spindle);
                num_spindles = c->spindle;
                break;

            case LBVMOT_SET_WORLD_HOME:
                log_print(
                    "SET_WORLD_HOME x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f, u=%.6f, v=%.6f, w=%.6f\n",
                    c->pos.tran.x, c->pos.tran.y, c->pos.tran.z,
                    c->pos.a, c->pos.b, c->pos.c,
                    c->pos.u, c->pos.v, c->pos.w
                );
                break;

            case LBVMOT_SET_JOINT_HOMING_PARAMS:
                log_print(
                    "SET_JOINT_HOMING_PARAMS joint=%d, offset=%.6f home=%.6f, final_vel=%.6f, search_vel=%.6f, latch_vel=%.6f, flags=0x%08x, sequence=%d, volatile=%d\n",
                    c->joint, c->offset, c->home, c->home_final_vel,
                    c->search_vel, c->latch_vel, c->flags,
                    c->home_sequence, c->volatile_home
                );
                break;

            case LBVMOT_UPDATE_JOINT_HOMING_PARAMS:
                log_print(
                    "UPDATE_JOINT_HOMING_PARAMS joint=%d, offset=%.6f home=%.6f home_sequence=%d\n",
                    c->joint, c->offset, c->home, c->home_sequence
                );
                break;

            case LBVMOT_SET_DEBUG:
                log_print("SET_DEBUG\n");
                break;

            case LBVMOT_SET_DOUT:
                log_print("SET_DOUT\n");
                break;

            case LBVMOT_SET_AOUT:
                log_print("SET_AOUT\n");
                break;

            case LBVMOT_SET_SPINDLESYNC:
                log_print("SET_SPINDLESYNC sync=%06f, flags=0x%08x\n", c->spindlesync, c->flags);
                break;

            case LBVMOT_SPINDLE_ON:
                log_print("SPINDLE_ON speed=%f, css_factor=%f, xoffset=%f\n", c->vel, c->ini_maxvel, c->acc);
                lbvmotStatus->spindle_status[0].speed = c->vel;
                break;

            case LBVMOT_SPINDLE_OFF:
                log_print("SPINDLE_OFF\n");
                lbvmotStatus->spindle_status[0].speed = 0;
                break;

            case LBVMOT_SPINDLE_INCREASE:
                log_print("SPINDLE_INCREASE\n");
                break;

            case LBVMOT_SPINDLE_DECREASE:
                log_print("SPINDLE_DECREASE\n");
                break;

            case LBVMOT_SPINDLE_BRAKE_ENGAGE:
                log_print("SPINDLE_BRAKE_ENGAGE\n");
                break;

            case LBVMOT_SPINDLE_BRAKE_RELEASE:
                log_print("SPINDLE_BRAKE_RELEASE\n");
                break;

            case LBVMOT_SPINDLE_ORIENT:
                log_print("SPINDLE_ORIENT\n");
                break;

            case LBVMOT_SET_JOINT_MOTOR_OFFSET:
                log_print("SET_JOINT_MOTOR_OFFSET\n");
                break;

            case LBVMOT_SET_JOINT_COMP:
                log_print("SET_JOINT_COMP\n");
                break;

            case LBVMOT_SET_OFFSET:
                log_print(
                    "SET_OFFSET x=%.6f, y=%.6f, z=%.6f, a=%.6f, b=%.6f, c=%.6f u=%.6f, v=%.6f, w=%.6f\n",
                    c->tool_offset.tran.x, c->tool_offset.tran.y, c->tool_offset.tran.z,
                    c->tool_offset.a, c->tool_offset.b, c->tool_offset.c,
                    c->tool_offset.u, c->tool_offset.v, c->tool_offset.w
                );
                break;

            case LBVMOT_SET_MAX_FEED_OVERRIDE:
                log_print("SET_MAX_FEED_OVERRIDE %.6f\n", c->maxFeedScale);
                break;

            case LBVMOT_SETUP_ARC_BLENDS:
                log_print("SETUP_ARC_BLENDS\n");
                break;

            case LBVMOT_SET_PROBE_ERR_INHIBIT:
                log_print("SETUP_SET_PROBE_ERR_INHIBIT %d %d\n",
                          c->probe_jog_err_inhibit,
                          c->probe_home_err_inhibit);
                break;


            default:
                log_print("ERROR: unknown command %d\n", c->command);
                break;
        }

        update_joint_status();

        lbvmotStatus->commandEcho = c->command;
        lbvmotStatus->commandNumEcho = c->commandNum;
        lbvmotStatus->commandStatus = LBVMOT_COMMAND_OK;
        lbvmotStatus->tail = lbvmotStatus->head;
    }

    return 0;
}

