/********************************************************************
* Description: lbvpose.h
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
********************************************************************/
#ifndef LBVPOSE_H
#define LBVPOSE_H

#include "lbvpos.h"
#include "posemath.h"

typedef enum {
    LBVPOSE_ERR_OK = 0,
    LBVPOSE_ERR_FAIL = -1,
    LBVPOSE_ERR_INPUT_MISSING = -2,
    LBVPOSE_ERR_OUTPUT_MISSING = -3,
    LBVPOSE_ERR_ALL
} LbvPoseErr;

void lbvPoseZero(LbvPose * const pos);

int lbvPoseAdd(LbvPose const * const p1, LbvPose const * const p2, LbvPose * const out);
int lbvPoseSub(LbvPose const * const p1, LbvPose const * const p2, LbvPose * const out);

int lbvPoseToPmCartesian(LbvPose const * const pose,
        PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw);
int pmCartesianToLbvPose(PmCartesian const * const xyz,
        PmCartesian const * const abc, PmCartesian const * const uvw, LbvPose * const pose);

int lbvPoseSelfAdd(LbvPose * const self, LbvPose const * const p2);
int lbvPoseSelfSub(LbvPose * const self, LbvPose const * const p2);

int lbvPoseSetXYZ(PmCartesian const * const xyz, LbvPose * const pose);
int lbvPoseSetABC(PmCartesian const * const abc, LbvPose * const pose);
int lbvPoseSetUVW(PmCartesian const * const uvw, LbvPose * const pose);

int lbvPoseGetXYZ(LbvPose const * const pose, PmCartesian * const xyz);
int lbvPoseGetABC(LbvPose const * const pose, PmCartesian * const abc);
int lbvPoseGetUVW(LbvPose const * const pose, PmCartesian * const uvw);

int lbvPoseMagnitude(LbvPose const * const pose, double * const out);

int lbvPoseValid(LbvPose const * const pose);

#endif
