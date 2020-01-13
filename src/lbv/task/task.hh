//    Copyright 2007 Jeff Epler <jepler@unpythonic.net>
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#ifndef LBV_TASK_HH
#define LBV_TASK_HH
#include "taskclass.hh"
extern NMLmsg *lbvTaskCommand;
extern int stepping;
extern int steppingWait;
extern int lbvTaskQueueCommand(NMLmsg *cmd);
extern int lbvPluginCall(LBV_EXEC_PLUGIN_CALL *call_msg);
extern int lbvIoPluginCall(LBV_IO_PLUGIN_CALL *call_msg);
extern int lbvTaskOnce(const char *inifile);
extern int lbvRunHalFiles(const char *filename);

// Returns 0 if all joints are homed, 1 if any joints are un-homed.
int all_homed(void);

int lbvTaskInit();
int lbvTaskHalt();
int lbvTaskAbort();
int lbvTaskSetMode(int mode);
int lbvTaskSetState(int state);
int lbvTaskPlanInit();
int lbvTaskPlanSetWait();
int lbvTaskPlanIsWait();
int lbvTaskPlanClearWait();
int lbvTaskPlanSynch();
int lbvTaskPlanSetOptionalStop(bool state);
int lbvTaskPlanSetBlockDelete(bool state);
void lbvTaskPlanExit();
int lbvTaskPlanOpen(const char *file);
int lbvTaskPlanRead();
int lbvTaskPlanExecute(const char *command);
int lbvTaskPlanExecute(const char *command, int line_number); //used in case of MDI to pass the pseudo line number to interp
int lbvTaskPlanPause();
int lbvTaskPlanResume();
int lbvTaskPlanClose();
int lbvTaskPlanReset();

int lbvTaskPlanLine();
int lbvTaskPlanLevel();
int lbvTaskPlanCommand(char *cmd);

int lbvTaskUpdate(LBV_TASK_STAT * stat);

#endif

