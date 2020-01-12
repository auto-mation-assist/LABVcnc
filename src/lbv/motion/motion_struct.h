/********************************************************************
* Description: motion_struct.h
*   A data structure used in only a few places
*
* Author:
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2004 All rights reserved
********************************************************************/

#ifndef MOTION_STRUCT_H
#define MOTION_STRUCT_H

/* big comm structure, for upper memory */
    typedef struct lbvmot_struct_t {
	struct lbvmot_command_t command;	/* struct used to pass commands/data
					   to the RT module from usr space */
	struct lbvmot_status_t status;	/* Struct used to store RT status */
	struct lbvmot_config_t config;	/* Struct used to store RT config */
	struct lbvmot_internal_t internal;	/*! \todo FIXME - doesn't need to be in
					   shared memory */
	struct lbvmot_error_t error;	/* ring buffer for error messages */
	struct lbvmot_debug_t debug;	/* Struct used to store RT status and debug
				   data - 2nd largest block */
    } lbvmot_struct_t;


#endif // MOTION_STRUCT_H
