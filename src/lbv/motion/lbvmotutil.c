/********************************************************************
* Description: lbvmotutil.c
*   Utility functions shared between motion and other systems
*
*   Derived from a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
********************************************************************/

#include "lbvmotcfg.h"		/* LBVMOT_ERROR_NUM,LEN */
#include "motion.h"		/* these decls */
#include "dbuf.h"
#include "stashf.h"

int lbvmotErrorInit(lbvmot_error_t * errlog)
{
    if (errlog == 0) {
	return -1;
    }

    errlog->head = 0;
    errlog->start = 0;
    errlog->end = 0;
    errlog->num = 0;
    errlog->tail = 0;

    return 0;
}

int lbvmotErrorPutfv(lbvmot_error_t * errlog, const char *fmt, va_list ap)
{
    struct dbuf errbuf;
    struct dbuf_iter it;

    if (errlog == 0 || errlog->num == LBVMOT_ERROR_NUM) {
	/* full */
	return -1;
    }

    errlog->head++;

    dbuf_init(&errbuf, (unsigned char*)errlog->error[errlog->end], LBVMOT_ERROR_LEN);
    dbuf_iter_init(&it, &errbuf);
    vstashf(&it, fmt, ap);

    errlog->end = (errlog->end + 1) % LBVMOT_ERROR_NUM;
    errlog->num++;

    errlog->tail = errlog->head;

    return 0;
}

int lbvmotErrorPutf(lbvmot_error_t *errlog, const char *fmt, ...)
{
    int result;
    va_list ap;
    va_start(ap, fmt);
    result = lbvmotErrorPutfv(errlog, fmt, ap);
    va_end(ap);
    return result;
}

int lbvmotErrorPut(lbvmot_error_t *errlog, const char *error)
{
    return lbvmotErrorPutf(errlog, "%s", error);
}

int lbvmotErrorGet(lbvmot_error_t * errlog, char *error)
{
    if (errlog == 0 || errlog->num == 0) {
	/* empty */
	return -1;
    }

    errlog->head++;
    memcpy(error, errlog->error[errlog->start], LBVMOT_ERROR_LEN);
    errlog->start = (errlog->start + 1) % LBVMOT_ERROR_NUM;
    errlog->num--;
    errlog->tail = errlog->head;

    return 0;
}
