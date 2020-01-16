/********************************************************************
* Description: iotaskintf.cc
*   NML interface functions for IO
*
*   Based on a work by Fred Proctor & Will Shackleford
*
* Author:
* License: GPL Version 2
* System: Linux
*    
* Copyright (c) 2004 All rights reserved.
*
* Last change:
********************************************************************/

#include <math.h>		// fabs()
#include <float.h>		// DBL_MAX
#include <string.h>		// memcpy() strncpy()
#include <stdlib.h>		// malloc()

#include "rcs.hh"		// RCS_CMD_CHANNEL, etc.
#include "rcs_print.hh"
#include "timer.hh"             // esleep, etc.
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"
#include "lbvglb.h"		// LBV_INIFILE

#include "initool.hh"

// IO INTERFACE

// the NML channels to the LBVIO controller
static RCS_CMD_CHANNEL *lbvIoCommandBuffer = 0;
static RCS_STAT_CHANNEL *lbvIoStatusBuffer = 0;

// global status structure
LBV_IO_STAT *lbvIoStatus = 0;

// serial number for communication
static int lbvIoCommandSerialNumber = 0;
static double LBVIO_BUFFER_GET_TIMEOUT = 5.0;

static int forceCommand(RCS_CMD_MSG *msg);

static int lbvioNmlGet()
{
    int retval = 0;
    double start_time;
    RCS_PRINT_DESTINATION_TYPE orig_dest;
    if (lbvIoCommandBuffer == 0) {
	orig_dest = get_rcs_print_destination();
	set_rcs_print_destination(RCS_PRINT_TO_NULL);
	start_time = etime();
	while (start_time - etime() < LBVIO_BUFFER_GET_TIMEOUT) {
	    lbvIoCommandBuffer =
		new RCS_CMD_CHANNEL(lbvFormat, "toolCmd", "lbv",
				    lbv_nmlfile);
	    if (!lbvIoCommandBuffer->valid()) {
		delete lbvIoCommandBuffer;
		lbvIoCommandBuffer = 0;
	    } else {
		break;
	    }
	    esleep(0.1);
	}
	set_rcs_print_destination(orig_dest);
    }

    if (lbvIoCommandBuffer == 0) {
	lbvIoCommandBuffer =
	    new RCS_CMD_CHANNEL(lbvFormat, "toolCmd", "lbv", lbv_nmlfile);
	if (!lbvIoCommandBuffer->valid()) {
	    delete lbvIoCommandBuffer;
	    lbvIoCommandBuffer = 0;
	    retval = -1;
	}
    }

    if (lbvIoStatusBuffer == 0) {
	orig_dest = get_rcs_print_destination();
	set_rcs_print_destination(RCS_PRINT_TO_NULL);
	start_time = etime();
	while (start_time - etime() < LBVIO_BUFFER_GET_TIMEOUT) {
	    lbvIoStatusBuffer =
		new RCS_STAT_CHANNEL(lbvFormat, "toolSts", "lbv",
				     lbv_nmlfile);
	    if (!lbvIoStatusBuffer->valid()) {
		delete lbvIoStatusBuffer;
		lbvIoStatusBuffer = 0;
	    } else {
		lbvIoStatus =
		    (LBV_IO_STAT *) lbvIoStatusBuffer->get_address();
		break;
	    }
	    esleep(0.1);
	}
	set_rcs_print_destination(orig_dest);
    }

    if (lbvIoStatusBuffer == 0) {
	lbvIoStatusBuffer =
	    new RCS_STAT_CHANNEL(lbvFormat, "toolSts", "lbv", lbv_nmlfile);
	if (!lbvIoStatusBuffer->valid()
	    || LBV_IO_STAT_TYPE != lbvIoStatusBuffer->peek()) {
	    delete lbvIoStatusBuffer;
	    lbvIoStatusBuffer = 0;
	    lbvIoStatus = 0;
	    retval = -1;
	} else {
	    lbvIoStatus = (LBV_IO_STAT *) lbvIoStatusBuffer->get_address();
	}
    }

    return retval;
}

static RCS_CMD_MSG *last_io_command = 0;
static long largest_io_command_size = 0;

/*
  sendCommand() waits until any currently executing command has finished,
  then writes the given command.*/
/*! \todo
  FIXME: Not very RCS-like to wait for status done here. (wps)
*/
static int sendCommand(RCS_CMD_MSG * msg)
{
    // need command buffer to be there
    if (0 == lbvIoCommandBuffer) {
	return -1;
    }
    // need status buffer also, to check for command received
    if (0 == lbvIoStatusBuffer || !lbvIoStatusBuffer->valid()) {
	return -1;
    }

    // always force-queue an abort
    if (msg->type == LBV_TOOL_ABORT_TYPE) {
	// just queue the abort and call it a day
	int rc = forceCommand(msg);
	if (rc) {
	    rcs_print_error("forceCommand(LBV_TOOL_ABORT) returned %d\n", rc);
	}
	return 0;
    }

    double send_command_timeout = etime() + 5.0;

    // check if we're executing, and wait until we're done
    int serial_diff = 0;
    while (etime() < send_command_timeout) {
	lbvIoStatusBuffer->peek();
	serial_diff = lbvIoStatus->echo_serial_number - lbvIoCommandSerialNumber;
	if (serial_diff < 0 || lbvIoStatus->status == RCS_EXEC) {
	    esleep(0.001);
	    continue;
	} else {
	    break;
	}
    }

    if (serial_diff < 0 || lbvIoStatus->status == RCS_EXEC) {
	// Still not done, must have timed out.
	rcs_print_error
	    ("Command to IO level (%s:%s) timed out waiting for last command done. \n",
	     lbvSymbolLookup(msg->type), lbvIoCommandBuffer->msg2str(msg));
	rcs_print_error
	    ("lbvIoStatus->echo_serial_number=%d, lbvIoCommandSerialNumber=%d, lbvIoStatus->status=%d\n",
	     lbvIoStatus->echo_serial_number, lbvIoCommandSerialNumber,
	     lbvIoStatus->status);
	if (0 != last_io_command) {
	    rcs_print_error("Last command sent to IO level was (%s:%s)\n",
			    lbvSymbolLookup(last_io_command->type),
			    lbvIoCommandBuffer->msg2str(last_io_command));
	}
	return -1;
    }
    // now we can send
    if (0 != lbvIoCommandBuffer->write(msg)) {
	rcs_print_error("Failed to send command to  IO level (%s:%s)\n",
			lbvSymbolLookup(msg->type),
			lbvIoCommandBuffer->msg2str(msg));
	return -1;
    }
    lbvIoCommandSerialNumber = msg->serial_number;

    if (largest_io_command_size < msg->size) {
	largest_io_command_size = std::max<long>(msg->size, 4096);
	last_io_command = (RCS_CMD_MSG *) realloc(last_io_command, largest_io_command_size);
    }

    if (0 != last_io_command) {
	memcpy(last_io_command, msg, msg->size);
    }

    return 0;
}

/*
  forceCommand() writes the given command regardless of the executing
  status of any previous command.
*/
static int forceCommand(RCS_CMD_MSG * msg)
{
    // need command buffer to be there
    if (0 == lbvIoCommandBuffer) {
	return -1;
    }
    // need status buffer also, to check for command received
    if (0 == lbvIoStatusBuffer || !lbvIoStatusBuffer->valid()) {
	return -1;
    }
    // send it immediately
    if (0 != lbvIoCommandBuffer->write(msg)) {
	rcs_print_error("Failed to send command to  IO level (%s:%s)\n",
			lbvSymbolLookup(msg->type),
			lbvIoCommandBuffer->msg2str(msg));
	return -1;
    }
    lbvIoCommandSerialNumber = msg->serial_number;

    if (largest_io_command_size < msg->size) {
	largest_io_command_size = std::max<long>(msg->size, 4096);
	last_io_command = (RCS_CMD_MSG *) realloc(last_io_command, largest_io_command_size);
    }

    if (0 != last_io_command) {
	memcpy(last_io_command, msg, msg->size);
    }

    return 0;
}

// NML commands

int lbvIoInit()
{
    LBV_TOOL_INIT ioInitMsg;

    // get NML buffer to lbvio
    if (0 != lbvioNmlGet()) {
	rcs_print_error("lbvioNmlGet() failed.\n");
	return -1;
    }

    if (0 != iniTool(lbv_inifile)) {
	return -1;
    }
    // send init command to lbvio
    if (forceCommand(&ioInitMsg)) {
	rcs_print_error("Can't forceCommand(ioInitMsg)\n");
	return -1;
    }

    return 0;
}

int lbvIoHalt()
{
    LBV_TOOL_HALT ioHaltMsg;

    // send halt command to lbvio
    if (lbvIoCommandBuffer != 0) {
	forceCommand(&ioHaltMsg);
    }
    // clear out the buffers

    if (lbvIoStatusBuffer != 0) {
	delete lbvIoStatusBuffer;
	lbvIoStatusBuffer = 0;
	lbvIoStatus = 0;
    }

    if (lbvIoCommandBuffer != 0) {
	delete lbvIoCommandBuffer;
	lbvIoCommandBuffer = 0;
    }

    if (last_io_command) {
        free(last_io_command);
        last_io_command = 0;
    }

    return 0;
}

int lbvIoAbort(int reason)
{
    LBV_TOOL_ABORT ioAbortMsg;

    ioAbortMsg.reason = reason;
    // send abort command to lbvio
    sendCommand(&ioAbortMsg);

    return 0;
}

int lbvIoSetDebug(int debug)
{
    LBV_SET_DEBUG ioDebugMsg;

    ioDebugMsg.debug = debug;

    return sendCommand(&ioDebugMsg);
}

int lbvAuxEstopOn()
{
    LBV_AUX_ESTOP_ON estopOnMsg;

    return forceCommand(&estopOnMsg);
}

int lbvAuxEstopOff()
{
    LBV_AUX_ESTOP_OFF estopOffMsg;

    return forceCommand(&estopOffMsg); //force the EstopOff message
}

int lbvCoolantMistOn()
{
    LBV_COOLANT_MIST_ON mistOnMsg;

    sendCommand(&mistOnMsg);

    return 0;
}

int lbvCoolantMistOff()
{
    LBV_COOLANT_MIST_OFF mistOffMsg;

    sendCommand(&mistOffMsg);

    return 0;
}

int lbvCoolantFloodOn()
{
    LBV_COOLANT_FLOOD_ON floodOnMsg;

    sendCommand(&floodOnMsg);

    return 0;
}

int lbvCoolantFloodOff()
{
    LBV_COOLANT_FLOOD_OFF floodOffMsg;

    sendCommand(&floodOffMsg);

    return 0;
}

int lbvLubeOn()
{
    LBV_LUBE_ON lubeOnMsg;

    sendCommand(&lubeOnMsg);

    return 0;
}

int lbvLubeOff()
{
    LBV_LUBE_OFF lubeOffMsg;

    sendCommand(&lubeOffMsg);

    return 0;
}

int lbvToolPrepare(int p, int tool)
{
    LBV_TOOL_PREPARE toolPrepareMsg;

    toolPrepareMsg.pocket = p;
    toolPrepareMsg.tool = tool;
    sendCommand(&toolPrepareMsg);

    return 0;
}


int lbvToolStartChange()
{
    LBV_TOOL_START_CHANGE toolStartChangeMsg;

    sendCommand(&toolStartChangeMsg);

    return 0;
}


int lbvToolLoad()
{
    LBV_TOOL_LOAD toolLoadMsg;

    sendCommand(&toolLoadMsg);

    return 0;
}

int lbvToolUnload()
{
    LBV_TOOL_UNLOAD toolUnloadMsg;

    sendCommand(&toolUnloadMsg);

    return 0;
}

int lbvToolLoadToolTable(const char *file)
{
    LBV_TOOL_LOAD_TOOL_TABLE toolLoadToolTableMsg;

    strcpy(toolLoadToolTableMsg.file, file);

    sendCommand(&toolLoadToolTableMsg);

    return 0;
}

int lbvToolSetOffset(int pocket, int toolno, LbvPose offset, double diameter,
                     double frontangle, double backangle, int orientation)
{
    LBV_TOOL_SET_OFFSET toolSetOffsetMsg;

    toolSetOffsetMsg.pocket = pocket;
    toolSetOffsetMsg.toolno = toolno;
    toolSetOffsetMsg.offset = offset;
    toolSetOffsetMsg.diameter = diameter;
    toolSetOffsetMsg.frontangle = frontangle;
    toolSetOffsetMsg.backangle = backangle;
    toolSetOffsetMsg.orientation = orientation;

    sendCommand(&toolSetOffsetMsg);

    return 0;
}

int lbvToolSetNumber(int number)
{
    LBV_TOOL_SET_NUMBER toolSetNumberMsg;

    toolSetNumberMsg.tool = number;

    sendCommand(&toolSetNumberMsg);

    return 0;
}

// Status functions

int lbvIoUpdate(LBV_IO_STAT * stat)
{

    if (0 == lbvIoStatusBuffer || !lbvIoStatusBuffer->valid()) {
	return -1;
    }

    switch (lbvIoStatusBuffer->peek()) {
    case -1:
	// error on CMS channel
	return -1;
	break;

    case 0:			// nothing new
    case LBV_IO_STAT_TYPE:	// something new
	// drop out to copy
	break;

    default:
	// something else is in there
	return -1;
	break;
    }

    // copy status
    *stat = *lbvIoStatus;

    /*
       We need to check that the RCS_DONE isn't left over from the previous
       command, by comparing the command number we sent with the command
       number that lbvio echoes. If they're different, then the command
       hasn't been acknowledged yet and the state should be forced to be
       RCS_EXEC. */
    int serial_diff = lbvIoStatus->echo_serial_number - lbvIoCommandSerialNumber;
    if (serial_diff < 0) {
	stat->status = RCS_EXEC;
    }
    //commented out because it keeps resetting the spindle speed to some odd value
    //the speed gets set by the IO controller, no need to override it here (io takes care of increase/decrease speed too)
    // stat->spindle.speed = spindleSpeed;

    return 0;
}
