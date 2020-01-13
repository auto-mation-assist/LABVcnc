/********************************************************************
* Description: lbvpose.c
*
*   Miscellaneous functions to handle LbvPose operations
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author: Robert W. Ellenberg
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2014 All rights reserved.
*
********************************************************************/

#include "lbvpose.h"
#include "posemath.h"
#include "rtapi_math.h"

//#define LBVPOSE_PEDANTIC

void lbvPoseZero(LbvPose * const pos) {
#ifdef LBVPOSE_PEDANTIC
    if(!pos) {
        return LBVPOSE_ERR_INPUT_MISSING;
    }
#endif

    pos->tran.x = 0.0;               
    pos->tran.y = 0.0;               
    pos->tran.z = 0.0;               
    pos->a = 0.0;                    
    pos->b = 0.0;                    
    pos->c = 0.0;                    
    pos->u = 0.0;                    
    pos->v = 0.0;                    
    pos->w = 0.0;
}


int lbvPoseAdd(LbvPose const * const p1, LbvPose const * const p2, LbvPose * const out)
{
#ifdef LBVPOSE_PEDANTIC
    if (!p1 || !p2) {
        return LBVPOSE_ERR_INPUT_MISSING;
    }
#endif

    pmCartCartAdd(&p1->tran, &p2->tran, &out->tran);
    out->a = p1->a + p2->a;
    out->b = p1->b + p2->b;
    out->c = p1->c + p2->c;
    out->u = p1->u + p2->u;
    out->v = p1->v + p2->v;
    out->w = p1->w + p2->w;
    return LBVPOSE_ERR_OK;
}

int lbvPoseSub(LbvPose const * const p1, LbvPose const * const p2, LbvPose * const out)
{
#ifdef LBVPOSE_PEDANTIC
    if (!p1 || !p2) {
        return LBVPOSE_ERR_INPUT_MISSING;
    }
#endif

    pmCartCartSub(&p1->tran, &p2->tran, &out->tran);
    out->a = p1->a - p2->a;
    out->b = p1->b - p2->b;
    out->c = p1->c - p2->c;
    out->u = p1->u - p2->u;
    out->v = p1->v - p2->v;
    out->w = p1->w - p2->w;
    return LBVPOSE_ERR_OK;

}

int lbvPoseSelfAdd(LbvPose * const self, LbvPose const * const p2)
{
    return lbvPoseAdd(self, p2, self);
}

int lbvPoseSelfSub(LbvPose * const self, LbvPose const * const p2)
{
    return lbvPoseSub(self, p2, self);
}

int lbvPoseToPmCartesian(LbvPose const * const pose,
        PmCartesian * const xyz, PmCartesian * const abc, PmCartesian * const uvw)
{

#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_INPUT_MISSING;
    } 
    if (!xyz | !abc || !uvw) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
#endif

    //Direct copy of translation struct for xyz
    *xyz = pose->tran;

    //Convert ABCUVW axes into 2 pairs of 3D lines
    abc->x = pose->a;
    abc->y = pose->b;
    abc->z = pose->c;

    uvw->x = pose->u;
    uvw->y = pose->v;
    uvw->z = pose->w;
    return LBVPOSE_ERR_OK;
}


/**
 * Collect PmCartesian elements into 9D LbvPose structure.
 */
int pmCartesianToLbvPose(PmCartesian const * const xyz,
        PmCartesian const * const abc, PmCartesian const * const uvw, LbvPose * const pose)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz || !abc || !uvw) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif
    //Direct copy of translation struct for xyz
    pose->tran = *xyz;

    pose->a = abc->x;
    pose->b = abc->y;
    pose->c = abc->z;

    pose->u = uvw->x;
    pose->v = uvw->y;
    pose->w = uvw->z;
    return LBVPOSE_ERR_OK;
}


int lbvPoseSetXYZ(PmCartesian const * const xyz, LbvPose * const pose)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif

    pose->tran.x = xyz->x;
    pose->tran.y = xyz->y;
    pose->tran.z = xyz->z;
    return LBVPOSE_ERR_OK;
}


int lbvPoseSetABC(PmCartesian const * const abc, LbvPose * const pose)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!abc) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif

    pose->a = abc->x;
    pose->b = abc->y;
    pose->c = abc->z;
    return LBVPOSE_ERR_OK;
}


int lbvPoseSetUVW(PmCartesian const * const uvw, LbvPose * const pose)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!uvw) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif

    pose->u = uvw->x;
    pose->v = uvw->y;
    pose->w = uvw->z;

    return LBVPOSE_ERR_OK;
}


int lbvPoseGetXYZ(LbvPose const * const pose, PmCartesian * const xyz)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!xyz) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif

    xyz->x = pose->tran.x;
    xyz->y = pose->tran.y;
    xyz->z = pose->tran.z;
    return LBVPOSE_ERR_OK;
}


int lbvPoseGetABC(LbvPose const * const pose, PmCartesian * const abc)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!abc) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif

    abc->x = pose->a;
    abc->y = pose->b;
    abc->z = pose->c;
    return LBVPOSE_ERR_OK;
}


int lbvPoseGetUVW(LbvPose const * const pose, PmCartesian * const uvw)
{
#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
    if (!uvw) {
        return LBVPOSE_ERR_INPUT_MISSING;
   }
#endif

    uvw->x = pose->u;
    uvw->y = pose->v;
    uvw->z = pose->w;

    return LBVPOSE_ERR_OK;
}


/**
 * Find the magnitude of an LbvPose position, treating it like a single vector.
 */
int lbvPoseMagnitude(LbvPose const * const pose, double * const out) {

#ifdef LBVPOSE_PEDANTIC
    if (!pose) {
        return LBVPOSE_ERR_INPUT_MISSING;
    }
    if (!out) {
        return LBVPOSE_ERR_OUTPUT_MISSING;
    }
#endif

    double mag = 0.0;
    mag += pmSq(pose->tran.x);
    mag += pmSq(pose->tran.y);
    mag += pmSq(pose->tran.z);
    mag += pmSq(pose->a);
    mag += pmSq(pose->b);
    mag += pmSq(pose->c);
    mag += pmSq(pose->u);
    mag += pmSq(pose->v);
    mag += pmSq(pose->w);
    mag = pmSqrt(mag);

    *out = mag;
    return LBVPOSE_ERR_OK;
}


/**
 * Return true for a numerically valid pose, or false for an invalid pose (or null pointer).
 */
int lbvPoseValid(LbvPose const * const pose)
{

    if (!pose || 
            isnan(pose->tran.x) ||
            isnan(pose->tran.y) ||
            isnan(pose->tran.z) ||
            isnan(pose->a) ||
            isnan(pose->b) ||
            isnan(pose->c) ||
            isnan(pose->u) ||
            isnan(pose->v) ||
            isnan(pose->w)) {
        return 0;
    } else {
        return 1;
    }
}
