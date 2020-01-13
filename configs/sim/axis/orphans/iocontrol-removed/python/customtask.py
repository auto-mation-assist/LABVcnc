#   This is a component of LabvCNC
#   Copyright 2011, 2012, 2013 Dewey Garrett <dgarrett@panix.com>,
#   Michael Haberler <git@mah.priv.at>
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#   This program is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with this program; if not, write to the Free Software
#   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
#
import os
import sys
import signal
import traceback

import lbvtask
import lbvcanon
import interpreter
import hal

try:
    import lbv
except ImportError:
    import labvcnc as lbv  # ini only

try:
    import cPickle as pickle
except ImportError:
    import pickle

try:
    from userfuncs import UserFuncs
except ImportError:
    from nulluserfuncs import UserFuncs

def debug():
    # interpreter.this isnt usable until after Interpreter.init has been called
    if hasattr(interpreter,'this'):
        return interpreter.this.debugmask &  0x00040000 # LBV_DEBUG_PYTHON_TASK
    return 


def handler(signum, frame):
    ''' controlled shut down
    after this, lbvIoHalt() will be called, too
    '''
    print "Python Task shutdown handler"
    # this handler overrides the handler in lbvtaskmain, so call that as well
    lbvtask.lbvtask_quit(signum)


class CustomTask(lbvtask.Task,UserFuncs):

    def __init__(self):
        signal.signal(signal.SIGINT, handler)
        signal.signal(signal.SIGTERM, handler)
        try:
            if debug(): print "py:  CustomTask()"
            lbvtask.Task.__init__(self)
            self.inifile = lbv.ini(lbvtask.ini_filename())
            self.tcpins = int(self.inifile.find("TOOL", "TASK_TOOLCHANGE_PINS") or 0)
            self.startchange_pins = int(self.inifile.find("TOOL", "TASK_START_CHANGE_PINS") or 0)
            self.fault_pins = int(self.inifile.find("TOOL", "TASK_TOOLCHANGE_FAULT_PINS") or 0)

            h = hal.component("iocontrol.0")
            h.newpin("coolant-flood", hal.HAL_BIT, hal.HAL_OUT)
            h.newpin("coolant-mist", hal.HAL_BIT, hal.HAL_OUT)

            h.newpin("lube-level", hal.HAL_BIT, hal.HAL_OUT)
            h.newpin("lube", hal.HAL_BIT, hal.HAL_OUT)

            h.newpin("lbv-enable-in", hal.HAL_BIT, hal.HAL_IN)
            h.newpin("user-enable-out", hal.HAL_BIT, hal.HAL_OUT)
            h.newpin("user-request-enable", hal.HAL_BIT, hal.HAL_OUT)

            if self.tcpins:
                h.newpin("tool-change", hal.HAL_BIT, hal.HAL_OUT)
                h.newpin("tool-changed", hal.HAL_BIT, hal.HAL_IN)
                h.newpin("tool-number", hal.HAL_S32, hal.HAL_OUT)
                h.newpin("tool-prep-number", hal.HAL_S32, hal.HAL_OUT)
                h.newpin("tool-prep-pocket", hal.HAL_S32, hal.HAL_OUT)
                h.newpin("tool-prepare", hal.HAL_BIT, hal.HAL_OUT)
                h.newpin("tool-prepared", hal.HAL_BIT, hal.HAL_IN)
            if self.startchange_pins:
                h.newpin("start-change", hal.HAL_BIT, hal.HAL_OUT)
                h.newpin("start-change-ack", hal.HAL_BIT, hal.HAL_IN)
            if self.fault_pins:
                h.newpin("lbv-abort", hal.HAL_BIT, hal.HAL_OUT)
                h.newpin("lbv-abort-ack", hal.HAL_BIT, hal.HAL_IN)
                h.newpin("lbv-reason", hal.HAL_S32, hal.HAL_OUT)
                h.newpin("toolchanger-fault", hal.HAL_BIT, hal.HAL_IN)
                h.newpin("toolchanger-fault-ack", hal.HAL_BIT, hal.HAL_OUT)
                h.newpin("toolchanger-reason", hal.HAL_S32, hal.HAL_IN)
                h.newpin("toolchanger-faulted", hal.HAL_BIT, hal.HAL_OUT)
                h.newpin("toolchanger-clear-fault", hal.HAL_BIT, hal.HAL_IN)

            h.ready()
            self.components = dict()
            self.components["iocontrol.0"] = h
            self.hal = h
            self.hal_init_pins()
            self.io = lbvtask.lbvstat.io
            self.io.aux.estop = 1
            self._callback = None
            self._check = None
            tt = self.io.tool.toolTable
            for p in range(0,len(tt)):
                tt[p].zero()
            UserFuncs.__init__(self)
            self.enqueue = EnqueueCall(self)
        except Exception,e:
            print "__init__"
            print_exc_plus()
            self.io.status  = lbvtask.RCS_STATUS.RCS_ERROR
        else:
            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE


    def lbvIoInit(self):
        if debug(): print "py:  lbvIoInit tt=",self.tooltable_filename
        try:
            self.io.aux.estop = 1
            self.io.tool.pocketPrepped = -1;
            self.io.tool.toolInSpindle = 0;
            self.io.coolant.mist = 0
            self.io.coolant.flood = 0
            self.io.lube.on = 0
            self.io.lube.level = 1

            self.hal_init_pins()
            # on nonrandom machines, always start by assuming the spindle is empty
            if not self.random_toolchanger:
                 self.io.tool.toolTable[0].zero()

            if self.inifile.find("TOOL", "ODBC_CONNECT"):
                import sqltoolaccess
                self.tt = sqltoolaccess.SqlToolAccess(self.inifile, self.random_toolchanger)
            else:
                import tooltable
                self.tt = tooltable.LbvToolTable(self.tooltable_filename, self.random_toolchanger)

            self.comments = dict()
            self.fms = dict()
            self.tt.load_table(self.io.tool.toolTable,self.comments,self.fms)
            self.tt.restore_state(lbvtask.lbvstat)
            # self.io.tool.toolInSpindle = 2 # works
            self.reload_tool_number(self.io.tool.toolInSpindle)

        except Exception,e:
            print "lbvIoInit",e
            print_exc_plus()
            self.io.status  = lbvtask.RCS_STATUS.RCS_ERROR
        else:
            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        finally:
            return 0

    def lbvToolLoadToolTable(self,file):
        # triggered by UI if tooltable was edited
        if debug(): print "py:  lbvToolLoadToolTable file = '%s'" % (file)
        self.comments = dict()
        self.fms = dict()
        try:
            self.tt.load_table(self.io.tool.toolTable,self.comments,self.fms)
        except Exception,e:
            print_exc_plus()
            self.io.status  = lbvtask.RCS_STATUS.RCS_ERROR
        else:
            self.reload_tool_number(self.io.tool.toolInSpindle)
            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def prepare_complete(self):
        if debug(): print "prepare complete"
        self.io.tool.pocketPrepped = self.hal["tool-prep-pocket"]
        self.hal["tool-prepare"] = 0

    def lbvToolPrepare(self,p,tool):
        if debug(): print "py:   lbvToolPrepare p =",p,"tool =",tool
        if self.random_toolchanger and (p == 0):
            if debug(): print "it doesn't make sense to prep the spindle pocket"
            return 0

        if self.tcpins:
            if self.fault_pins and self.hal["toolchanger-faulted"]:
                if debug(): print "prepare: toolchanger faulted (reason=%d), next M6 will %s" % (self.hal["toolchanger-reason"], "set fault code and reason"  if self.hal["toolchanger-reason"] > 0 else "abort program")
            self.hal["tool-prep-pocket"] = p
            if not self.random_toolchanger and (p == 0):
                self.hal["tool-prep-number"] = 0
            else:
                self.hal["tool-prep-number"] = self.io.tool.toolTable[p].toolno

                self.hal["tool-prepare"] = 1

                # and tell task to wait until status changes to RCS_DONE
                self.io.status =  self.wait_for_named_pin(1,"iocontrol.0.tool-prepared",self.prepare_complete)
        else:
            self.io.tool.pocketPrepped = p
            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def reload_tool_number(self, toolno):
        if self.random_toolchanger: return
        t = self.io.tool.toolTable
        for p in range(1,len(t)):
            if toolno == t[p].toolno:
                self.load_tool(p)

    def load_tool(self,pocket):
        if self.random_toolchanger:
            self.io.tool.toolTable[0],self.io.tool.toolTable[pocket] = self.io.tool.toolTable[pocket],self.io.tool.toolTable[0]
            self.comments[0],self.comments[pocket] = self.comments[pocket],self.comments[0]
            self.tt.save_table(self.io.tool.toolTable,self.comments,self.fms)
        else:
            if pocket == 0:
                self.io.tool.toolTable[0].zero()
            else:
                self.io.tool.toolTable[0] = self.io.tool.toolTable[pocket]

    def change_complete(self):
        if debug(): print "change complete"
        if not self.random_toolchanger and (self.io.tool.pocketPrepped == 0):
            self.io.tool.toolInSpindle = 0
        else:
            self.io.tool.toolInSpindle = self.io.tool.toolTable[self.io.tool.pocketPrepped].toolno
        self.hal["tool-number"] = self.io.tool.toolInSpindle
        self.load_tool(self.io.tool.pocketPrepped)
        self.io.tool.pocketPrepped = -1
        self.hal["tool-prep-number"] = 0
        self.hal["tool-prep-pocket"] = 0
        self.hal["tool-change"] = 0

    def lbvToolLoad(self):
        if debug(): print "py:  lbvToolLoad"

        if self.random_toolchanger and (self.io.tool.pocketPrepped == 0):
            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
            return 0

        if not self.random_toolchanger and (self.io.tool.pocketPrepped > 0) and (
            self.io.tool.toolInSpindle ==
            self.io.tool.toolTable[self.io.tool.pocketPrepped].toolno):

            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
            return 0

        if self.tcpins:
            if self.fault_pins and self.hal["toolchanger-faulted"]:
                self.io.status  = lbvtask.RCS_STATUS.RCS_ERROR
                return 0
            if self.io.tool.pocketPrepped != -1:
                self.hal["tool-change"] = 1
                self.io.status =  self.wait_for_named_pin(1,"iocontrol.0.tool-changed",self.change_complete)
                return 0
        else:
            if not self.random_toolchanger and (self.io.tool.pocketPrepped == 0):
                self.io.tool.toolInSpindle = 0
            else:
                self.io.tool.toolInSpindle = self.io.tool.toolTable[self.io.tool.pocketPrepped].toolno
            self.load_tool(self.io.tool.pocketPrepped)
            self.io.tool.pocketPrepped = -1
            self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvToolUnload(self):
        if debug(): print "py:  lbvToolUnload"
        self.io.tool.toolInSpindle = 0
        # this isnt in ioControlv1, but I think it should be.
        self.hal["tool-number"] = self.io.tool.toolInSpindle
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvToolSetNumber(self,number):
        if debug(): print "py:   lbvToolSetNumber number =",number
        self.io.tool.toolInSpindle = number
        if self.tcpins:
            self.hal["tool-number"] = self.io.tool.toolInSpindle
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvToolSetOffset(self,pocket,toolno,offset,diameter,frontangle,backangle,orientation):
        if debug(): print "py:  lbvToolSetOffset", pocket,toolno,str(offset),diameter,frontangle,backangle,orientation

        self.io.tool.toolTable[pocket].toolno = toolno
        self.io.tool.toolTable[pocket].orientation = orientation
        self.io.tool.toolTable[pocket].diameter = diameter
        self.io.tool.toolTable[pocket].frontangle = frontangle
        self.io.tool.toolTable[pocket].backangle = backangle
        self.io.tool.toolTable[pocket].offset = offset

        if debug(): print "new tool enttry: ",str(self.io.tool.toolTable[pocket])

        if self.io.tool.toolInSpindle  == toolno:
            self.io.tool.toolTable[0] = self.io.tool.toolTable[pocket]

        self.tt.save_table(self.io.tool.toolTable,self.comments,self.fms)
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0


    def lbvIoPluginCall(self, len, msg):
        if debug(): print "py: lbvIoPluginCall len=%d msg=%s" %(len,msg)
        call = pickle.loads(msg)
        func = getattr(self, call[0], None)
        if func:
            self.io.status = func(*call[1],**call[2])
        else:
            raise AttributeError, "no such method: " + call[0]
        return 0


    def lbvIoHalt(self):
        if debug(): print "py:  lbvIoHalt"
        self.tt.save_state(lbvtask.lbvstat)
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbv_abort_acked(self):
        if debug(): print "lbv_abort_acked"
        self.hal["lbv-abort"] = 0

    def lbvIoAbort(self,reason):
        if debug(): print "py:  lbvIoAbort reason=",reason,"state=",lbvtask.lbvstat.task.state
        #if debug(): print "tc fault=",self.io.fault, "tc reason=",self.io.reason

        self.io.coolant.mist = 0
        self.io.coolant.flood = 0

        if self.tcpins:
            self.hal["coolant-mist"] = 0
            self.hal["coolant-flood"] = 0
            self.hal["tool-change"] = 0
            self.hal["tool-prepare"] = 0
        if self.startchange_pins:
            self.hal["start-change"] = 0
        if self.fault_pins:
            self.hal["lbv-reason"] = reason
            self.hal["lbv-abort"] = 1
            self.io.status =  self.wait_for_named_pin(1,"iocontrol.0.lbv-abort-ack",self.lbv_abort_acked)
            return 0

        if self._callback:
            if debug(): print "lbvIoAbort: cancelling callback to ",self._callback
            self._callback = None
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def start_change_acked(self):
        if debug(): print "start_change_acked"
        self.hal["start-change"] = 0

    def lbvToolStartChange(self):
        if debug(): print "py:  lbvToolStartChange", "wait for iocontrol.0.start-change-ack" if self.startchange_pins else "noop"
        if self.startchange_pins:
                self.hal["start-change"] = 1
                self.io.status =  self.wait_for_named_pin(1,"iocontrol.0.start-change-ack",self.start_change_acked)
                return 0
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvAuxEstopOn(self):
        if debug(): print "py:  lbvAuxEstopOn taskstate=",lbvtask.lbvstat.task.state
        self.hal["user-enable-out"] = 0
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvAuxEstopOff(self):
        if debug(): print "py:  lbvAuxEstopOff"
        self.hal["user-enable-out"] = 1
        self.hal["user-request-enable"] = 1
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvCoolantMistOn(self):
        if debug(): print "py:  lbvCoolantMistOn"
        self.hal["coolant-mist"] = 1
        self.io.coolant.mist = 1
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvCoolantMistOff(self):
        if debug(): print "py:  lbvCoolantMistOff"
        self.hal["coolant-mist"] = 0
        self.io.coolant.mist = 0
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvCoolantFloodOn(self):
        if debug(): print "py:  lbvCoolantFloodOn"
        self.hal["coolant-flood"] = 1
        self.io.coolant.flood = 1
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvCoolantFloodOff(self):
        if debug(): print "py:  lbvCoolantFloodOff"
        self.hal["coolant-flood"] = 0
        self.io.coolant.flood = 0
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvLubeOn(self):
        if debug(): print "py:  lbvLubeOn"
        self.hal["lube"] = 1
        self.io.lube.on = 1
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvLubeOff(self):
        if debug(): print "py:  lbvLubeOff"
        self.hal["lube"] = 0
        self.io.lube.on = 0
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvIoSetDebug(self,debug):
        if debug(): print "py:   lbvIoSetDebug debug =",debug
        self.io.status  = lbvtask.RCS_STATUS.RCS_DONE
        return 0

    def lbvIoUpdate(self):
        try:
            #if debug(): print "py:  lbvIoUpdate"
            self.hal["user-request-enable"] = 0
            self.io.aux.estop = not self.hal["lbv-enable-in"]
            if self.fault_pins:
                if self.hal["toolchanger-fault"]:
                    self.io.reason = self.hal["toolchanger-reason"]
                    self.hal["toolchanger-fault-ack"] = 1
                    self.hal["toolchanger-faulted"] = 1 # fault indicator latch
                    self.io.fault = 1
                    return 0
                else:
                    self.hal["toolchanger-fault-ack"] = 0
                if self.hal["toolchanger-clear-fault"]:
                    self.hal["toolchanger-faulted"] = 0 # reset fault indicator latch
                    self.io.reason = 0
            if self._check:
                self.io.status  = self._check()
                return 0
        except KeyboardInterrupt:  # shutting down
            print "lbvIoUpdate----KeyboardInterrupt:"
            return -1
        except Exception, e:
            print "lbvIoUpdate----:"
            print_exc_plus()
            return -1
        else:
            return 0

    def wait_for_named_pin_callback(self):
        if self._comp[self._pin] == self._value:
            if debug(): print "pin %s now %d" % (self._pin,  self._value)
            if self._callback: self._callback()
            self._check = None
            self._callback = None
            return lbvtask.RCS_STATUS.RCS_DONE
        return lbvtask.RCS_STATUS.RCS_EXEC

    def wait_for_named_pin(self,value,name,callback = None):
        (component, pin) = name.rsplit('.',1)
        comp = self.components[component]
        if comp[pin] == value:
            if debug(): print "pin: %s already at %d" % (name,value)
            if callback: callback()
            return lbvtask.RCS_STATUS.RCS_DONE
        else:
            if debug(): print "waiting for %s to become %d" % (name,value)
        # else set up callback
        self._comp = comp
        self._pin = pin
        self._value = value
        self._check = self.wait_for_named_pin_callback
        self._callback = callback
        # and tell task to wait until status changes to RCS_DONE
        return lbvtask.RCS_STATUS.RCS_EXEC


    def hal_init_pins(self):
        """ Sets HAL pins default values """
        self.hal["user-enable-out"] = 0
        self.hal["user-request-enable"] = 0
        self.hal["coolant-mist"] = 0
        self.hal["coolant-flood"] = 0
        self.hal["lube"] = 0
        if self.tcpins:
            self.hal["tool-prepare"] = 0
            self.hal["tool-prepared"] = 0
            self.hal["tool-prep-number"] = 0
            self.hal["tool-prep-pocket"] = 0
            self.hal["tool-change"] = 0
            self.hal["tool-number"] = 0
        if self.startchange_pins:
            self.hal["start-change"] = 0
        if self.fault_pins:
            self.hal["lbv-abort"] = 0
            self.hal["lbv-reason"] = 0
            self.hal["toolchanger-fault-ack"] = 0
            self.hal["toolchanger-faulted"] = 0


# support queuing calls from Interp to Task Python methods:
# trap call, pickle a tuple of name and arguments and enqueue with canon IO_PLUGIN_CALL
class EnqueueCall(object):
    def __init__(self,e):
        if debug(): print "EnqueueCall.__init__()"
        self._e = e

    def _encode(self,*args,**kwargs):
        if hasattr(self._e,self._name) and callable(getattr(self._e,self._name)):
            p = pickle.dumps((self._name,args,kwargs),-1) #  binary pickle
            lbvcanon.IO_PLUGIN_CALL(int(len(p)),p)
        else:
            raise AttributeError,"no such Task method: " + self._name

    def __getattr__(self, name):
        self._name = name
        return self._encode

## {{{ http://code.activestate.com/recipes/52215/ (r1)

def print_exc_plus():
    """
    Print the usual traceback information, followed by a listing of all the
    local variables in each frame.
    """
    tb = sys.exc_info()[2]
    while 1:
        if not tb.tb_next:
            break
        tb = tb.tb_next
    stack = []
    f = tb.tb_frame
    while f:
        stack.append(f)
        f = f.f_back
    stack.reverse()
    traceback.print_exc()
    print "Locals by frame, innermost last"
    for frame in stack:
        print
        print "Frame %s in %s at line %s" % (frame.f_code.co_name,
                                             frame.f_code.co_filename,
                                             frame.f_lineno)
        for key, value in frame.f_locals.items():
            print "\t%20s = " % key,
            #We have to be careful not to cause a new error in our error
            #printer! Calling str() on an unknown object could cause an
            #error we don't want.
            try:
                print value
            except:
                print "<ERROR WHILE PRINTING VALUE>"
