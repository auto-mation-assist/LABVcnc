#!/usr/bin/env python
# vim: sts=4 sw=4 et
# GladeVcp actions
#
# Copyright (c) 2010  Pavel Shramov <shramov@mexmat.net>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import gobject
import gtk
import os
import time
import re, string

from hal_widgets import _HalWidgetBase
import labvcnc
from hal_glib import GStat

_ = lambda x: x

class _LBVStaticHolder:
    def __init__(self):
        # Delay init...
        self.labvcnc = None
        self.stat = None
        self.gstat = None

    def get(self):
        if not self.labvcnc:
            self.labvcnc = labvcnc.command()
        if not self.gstat:
            self.gstat = GStat()
        return self.labvcnc, self.gstat.stat, self.gstat

class _LBVStatic:
    holder = _LBVStaticHolder()
    def get(self):
        return self.holder.get()

class _LBV_ActionBase(_HalWidgetBase):
    _gproperties = {'name': (gobject.TYPE_STRING, 'Name', 'Action name', "",
                             gobject.PARAM_READWRITE|gobject.PARAM_CONSTRUCT)
                   }

    labvcnc_static = _LBVStatic()

    def _hal_init(self):
        self.labvcnc, self.stat, self.gstat = self.labvcnc_static.get()
        self._stop_emission = False
        # if 'NO_FORCE_HOMING' is true, MDI  commands are allowed before homing.
        inifile = os.environ.get('INI_FILE_NAME', '/dev/null')
        ini = labvcnc.ini(inifile)
        self.no_f_home = int(ini.find("TRAJ", "NO_FORCE_HOMING") or 0)

    def machine_on(self):
        self.stat.poll()
        return self.stat.task_state > labvcnc.STATE_OFF

    def is_auto_mode(self):
        self.stat.poll()
        print self.stat.task_mode, labvcnc.MODE_AUTO
        return self.stat.task_mode == labvcnc.MODE_AUTO

    def is_file_loaded(self):
        self.stat.poll()
        print "file name:",self.stat.file
        if self.stat.file:
            return True
        else:
            return False

    def is_all_homed(self):
        self.stat.poll()
        homed_count = 0
        for i,h in enumerate(self.stat.homed):
            #Don't worry about joint to axis mapping
            if h: homed_count +=1
        print self.stat.joints
        if homed_count == self.stat.joints:
            return True
        return False

    def no_home_required(self):
        return self.no_f_home

    def safe_handler(self, f):
        def _f(self, *a, **kw):
            if self._stop_emission:
                return
            return f(self, *a, **kw)
        return _f

    def set_active_safe(self, active):
        self._stop_emission = True
        self.set_active(active)
        self._stop_emission = False

    def do_get_property(self, property):
        name = property.name.replace('-', '_')

        if name == 'name':
            return self.get_name()
        elif name == 'label':
            return self.get_label()
        elif name == 'tooltip':
            return self.get_tooltip()
        elif name == 'stock_id':
            return self.get_stock_id()
        else:
            raise AttributeError("Unknown property: %s" % property.name)

    def do_set_property(self, property, value):
        name = property.name.replace('-', '_')

        if name == 'name':
            if value:
                self.set_name(value)
        elif name == 'label':
            self.set_label(value)
        elif name == 'tooltip':
            self.set_tooltip(value)
        elif name == 'stock_id':
            self.set_stock_id(value)
        else:
            raise AttributeError("Unknown property: %s" % property.name)
        return True

class _LBV_Action(gtk.Action, _LBV_ActionBase):
    __gproperties__ = _LBV_ActionBase._gproperties
    def __init__(self, name=None):
        gtk.Action.__init__(self, None, None, None, None)
        self._stop_emission = False
        self.connect('activate', self.safe_handler(self.on_activate))

    def set_active_safe(self, a): return #XXX: Override set_active with nop

    def on_activate(self, w):
        return True

class _LBV_ToggleAction(gtk.ToggleAction, _LBV_ActionBase):
    __gproperties__ = _LBV_ActionBase._gproperties
    def __init__(self, name=None):
        gtk.ToggleAction.__init__(self, None, None, None, None)
        self._stop_emission = False
        self.connect('toggled', self.safe_handler(self.on_toggled))

    # XXX: Override nop in _LBV_Action
    set_active_safe = _LBV_ActionBase.set_active_safe

    def on_toggled(self, w):
        return True

class _LBV_RadioAction(gtk.RadioAction, _LBV_ToggleAction):
    __gproperties__ = _LBV_ToggleAction._gproperties
    def __init__(self, name=None):
        gtk.RadioAction.__init__(self, None, None, None, None, 0)
        self._stop_emission = False
        self.connect('toggled', self.safe_handler(self.on_toggled))

    def on_toggled(self, w):
        if not w.get_active():
            return
        return self.on_activate(w)

class LBV_Stat(GStat, _LBV_ActionBase):
    __gtype_name__ = 'LBV_Stat'
    def __init__(self):
        stat = self.labvcnc_static.get()[1]
        GStat.__init__(self, stat)

    def _hal_init(self):
        pass

def _action(klass, f, *a, **kw):
    class _C(_LBV_Action):
        __gtype_name__ = klass
        def on_activate(self, w):
            print klass
            f(self, *a, **kw)
    return _C

LBV_Action_ESTOP = _action('LBV_Action_ESTOP', lambda s: s.labvcnc.state(labvcnc.STATE_ESTOP))
LBV_Action_ESTOP_RESET = _action('LBV_Action_ESTOP_RESET', lambda s: s.labvcnc.state(labvcnc.STATE_ESTOP_RESET))
LBV_Action_ON    = _action('LBV_Action_ON', lambda s: s.labvcnc.state(labvcnc.STATE_ON))
LBV_Action_OFF   = _action('LBV_Action_OFF', lambda s: s.labvcnc.state(labvcnc.STATE_OFF))

class LBV_ToggleAction_ESTOP(_LBV_ToggleAction):
    __gtype_name__ = 'LBV_ToggleAction_ESTOP'
    def _hal_init(self):
        _LBV_ToggleAction._hal_init(self)

        self.set_active_safe(True)

        self.gstat.connect('state-estop', lambda w: self.set_active_safe(True))
        self.gstat.connect('state-estop-reset', lambda w: self.set_active_safe(False))

    def on_toggled(self, w):
        if self.get_active():
            print 'Issuing ESTOP'
            self.labvcnc.state(labvcnc.STATE_ESTOP)
        else:
            print 'Issuing ESTOP RESET'
            self.labvcnc.state(labvcnc.STATE_ESTOP_RESET)

class LBV_ToggleAction_Power(_LBV_ToggleAction):
    __gtype_name__ = 'LBV_ToggleAction_Power'
    def _hal_init(self):
        _LBV_ToggleAction._hal_init(self)

        self.set_active_safe(False)
        self.set_sensitive(False)

        self.gstat.connect('state-on',  lambda w: self.set_active_safe(True))
        self.gstat.connect('state-off', lambda w: self.set_active_safe(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop-reset', lambda w: self.set_sensitive(True))

    def on_toggled(self, w):
        if self.get_active():
            print 'Issuing ON'
            self.labvcnc.state(labvcnc.STATE_ON)
        else:
            print 'Issuing OFF'
            self.labvcnc.state(labvcnc.STATE_OFF)

class LBV_RadioAction_ESTOP(_LBV_RadioAction):
    __gtype_name__ = 'LBV_RadioAction_ESTOP'
    def _hal_init(self):
        _LBV_RadioAction._hal_init(self)

        self.set_active_safe(True)

        self.gstat.connect('state-estop', lambda w: self.set_active_safe(True))

    def on_activate(self, w):
        self.labvcnc.state(labvcnc.STATE_ESTOP)

class LBV_RadioAction_ESTOP_RESET(_LBV_RadioAction):
    __gtype_name__ = 'LBV_RadioAction_ESTOP_RESET'
    def _hal_init(self):
        _LBV_RadioAction._hal_init(self)

        self.set_active_safe(False)

        self.gstat.connect('state-estop-reset', lambda w: self.set_active_safe(True))

    def on_activate(self, w):
        self.labvcnc.state(labvcnc.STATE_ESTOP_RESET)

class LBV_RadioAction_ON(_LBV_RadioAction):
    __gtype_name__ = 'LBV_RadioAction_ON'
    def _hal_init(self):
        _LBV_RadioAction._hal_init(self)

        self.set_active_safe(True)

        self.gstat.connect('state-on',  lambda w: self.set_active_safe(True))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop-reset', lambda w: self.set_sensitive(True))

    def on_activate(self, w):
        self.labvcnc.state(labvcnc.STATE_ON)

class LBV_RadioAction_OFF(_LBV_RadioAction):
    __gtype_name__ = 'LBV_RadioAction_OFF'
    def _hal_init(self):
        _LBV_RadioAction._hal_init(self)

        self.set_active_safe(False)

        self.gstat.connect('state-off', lambda w: self.set_active_safe(True))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop-reset', lambda w: self.set_sensitive(True))

    def on_activate(self, w):
        self.labvcnc.state(labvcnc.STATE_OFF)

def running(s, do_poll=True):
    if do_poll: s.poll()
    return s.task_mode == labvcnc.MODE_AUTO and s.interp_state != labvcnc.INTERP_IDLE

def ensure_mode(s, c, *modes):
    s.poll()
    if not modes: return False
    if s.task_mode in modes: return True
    if running(s, do_poll=False): return False
    c.mode(modes[0])
    c.wait_complete()
    return True

class LBV_Action_Python(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Python'
    command = gobject.property(type=str, default='', nick='Python Command')
    is_homed = gobject.property(type=bool, default=True, nick='Must Be Homed',
                                    blurb='Machine Must be homed for widgets to be sensitive to input')
    is_on = gobject.property(type=bool, default=True, nick='Must Be On',
                                    blurb='Machine Must be On for widgets to be sensitive to input')
    is_idle = gobject.property(type=bool, default=True, nick='Must Be Idle',
                                    blurb='Machine Must be Idle for widgets to be sensitive to input')
    requires_manual = gobject.property(type=bool, default=False, nick='Preset Manual Mode',
                                    blurb='Preset Manual Mode before command')
    requires_mdi = gobject.property(type=bool, default=False, nick='Preset MDI Mode',
                                    blurb='Preset MDI Mode before command')
    requires_auto = gobject.property(type=bool, default=False, nick='Preset Auto Mode',
                                    blurb='Preset Auto Mode before command')

    def _hal_init(self):
        _LBV_Action._hal_init(self)
        self.set_sensitive(False)
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        if self.is_on:
            self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        if self.is_homed:
            self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on() and ( self.is_all_homed() or self.no_home_required() ) ))
        else:
            self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on()) )
        if self.is_idle:
            self.gstat.connect('interp-run', lambda w: self.set_sensitive(False))
        if self.is_homed:
            self.gstat.connect('all-homed', lambda w: self.set_sensitive(self.machine_on()))

    def on_activate(self, w):
        self._globalParameter = { 'EXT':self._panel_instance.get_handler_obj(),
                                  'GSTAT':self.gstat,'CMD':self.labvcnc,'STAT':self.stat}
        self._localsParameter = {'dir': dir, 'self': self,'labvcnc':labvcnc}
        if self.requires_manual:
            ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_MAN)
        elif self.requires_mdi:
            ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_MDI)
        elif self.requires_auto:
            ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_AUTO)
        exec(self.command, self._globalParameter, self._localsParameter)

class LBV_Action_Run(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Run'
    program_start_line = gobject.property(type=int, default=0, minimum=0, nick='Restart line',
                                    blurb='Restart line number-Usually 0 - program start')
    reset_line = gobject.property(type=int, default=0, minimum=0, nick='Restart line after restarting once',
                                    blurb='Line number that will be set afterthe next restart. -usually 0 - program start')

    def set_restart_line(self,line,resetline=0):
        self.program_start_line = line
        self.reset_line = resetline

    def on_activate(self, w):
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_AUTO)
        self.labvcnc.auto(labvcnc.AUTO_RUN, self.program_start_line)
        self.program_start_line = self.reset_line

class LBV_Action_Step(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Step'
    def _hal_init(self):
        _LBV_Action._hal_init(self)

        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on()))

    def on_activate(self, w):
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_AUTO)
        self.labvcnc.auto(labvcnc.AUTO_STEP)

class LBV_Action_Pause(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Pause'
    def on_activate(self, w):
        self.stat.poll()
        if self.stat.task_mode != labvcnc.MODE_AUTO or\
                self.stat.interp_state not in (labvcnc.INTERP_READING, labvcnc.INTERP_WAITING):
            return
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_AUTO)
        self.labvcnc.auto(labvcnc.AUTO_PAUSE)

class LBV_Action_Resume(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Resume'
    def on_activate(self, w):
        print "RESUME"
        self.stat.poll()
        if not self.stat.paused:
            return
        if self.stat.task_mode not in (labvcnc.MODE_AUTO, labvcnc.MODE_MDI):
            return
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_AUTO, labvcnc.MODE_MDI)
        self.labvcnc.auto(labvcnc.AUTO_RESUME)

class LBV_Action_Stop(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Stop'
    def on_activate(self, w):
        self.labvcnc.abort()
        self.labvcnc.wait_complete()

class LBV_ToggleAction_Run(_LBV_ToggleAction, LBV_Action_Run):
    __gtype_name__ = 'LBV_ToggleAction_Run'
    program_start_line = gobject.property(type=int, default=0, minimum=0, nick='Restart line',
                                    blurb='Restart line number-Usually 0 - program start')
    reset_line = gobject.property(type=int, default=0, minimum=0, nick='Restart line after restarting once',
                                    blurb='Line number that will be set afterthe next restart. -usually 0 - program start')
    def _hal_init(self):
        _LBV_ToggleAction._hal_init(self)

        self.set_active_safe(False)
        self.set_sensitive(False)

        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))

        self.gstat.connect( 'interp-idle', lambda w: self.set_sensitive( self.machine_on() and ( self.is_all_homed() or self.no_home_required() ) and self.is_file_loaded() ) )
        self.gstat.connect('interp-idle', lambda w: self.set_active_safe(False))
        self.gstat.connect('interp-run', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-run', lambda w: self.set_active_safe(True))
        self.gstat.connect('all-homed', lambda w: self.set_sensitive( self.machine_on() and self.is_file_loaded() ))
        self.gstat.connect('file-loaded', self.file_loaded_check)

    def file_loaded_check(self,widget,filename):
        self.set_sensitive( self.machine_on() and (self.is_all_homed() or self.no_home_required()) )

    def set_restart_line(self,line,resetline=0):
        self.program_start_line = line
        self.reset_line = resetline

    def on_toggled(self, w):
        if self.get_active():
            return self.on_activate(w)

class LBV_ToggleAction_Stop(_LBV_ToggleAction, LBV_Action_Stop):
    __gtype_name__ = "LBV_ToggleAction_Stop"
    def _hal_init(self):
        _LBV_ToggleAction._hal_init(self)

        self.set_active_safe(True)
        self.set_sensitive(False)

        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))

        self.gstat.connect('interp-idle', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-idle', lambda w: self.set_active_safe(True))
        self.gstat.connect('interp-run', lambda w: self.set_sensitive(self.machine_on()))
        self.gstat.connect('interp-run', lambda w: self.set_active_safe(False))

    def on_toggled(self, w):
        if self.get_active():
            return self.on_activate(w)

class LBV_ToggleAction_Pause(_LBV_ToggleAction, LBV_Action_Pause):
    __gtype_name__ = "LBV_ToggleAction_Pause"
    def _hal_init(self):
        _LBV_ToggleAction._hal_init(self)

        self.resume = LBV_Action_Resume()
        self.resume._hal_init()

        self.set_active_safe(True)
        self.set_sensitive(False)

        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))

        self.gstat.connect('interp-idle', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-idle', lambda w: self.set_active_safe(False))
        self.gstat.connect('interp-run', lambda w: self.set_sensitive(self.machine_on()))
        self.gstat.connect('interp-run', lambda w: self.set_active_safe(False))
        self.gstat.connect('interp-paused', lambda w: self.set_active_safe(True))
        self.gstat.connect('interp-waiting', lambda w: self.set_active_safe(False))

    def on_toggled(self, w):
        if self.get_active():
            return self.on_activate(w)
        else:
            return self.resume.on_activate(self.resume)

class HalTemplate(string.Template):
    idpattern = '[_a-z][-._a-z0-9]*'

class FloatComp:
    def __init__(self, comp):
        self.comp = comp
    def __getitem__(self, k):
        v = float(self.comp[k])
        return "%f" % v

class LBV_Action_MDI(_LBV_Action):
    __gtype_name__ = 'LBV_Action_MDI'
    command = gobject.property(type=str, default='', nick='MDI Command')

    def _hal_init(self):
        _LBV_Action._hal_init(self)
        self.set_sensitive(False)
        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on() and ( self.is_all_homed() or self.no_home_required() ) ))
        self.gstat.connect('interp-run', lambda w: self.set_sensitive(False))
        self.gstat.connect('all-homed', lambda w: self.set_sensitive(self.machine_on()))

    def on_activate(self, w):
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_MDI)
        template = HalTemplate(self.command)
        cmd = template.substitute(FloatComp(self.hal))
        self.labvcnc.mdi(cmd)

class LBV_ToggleAction_MDI(_LBV_ToggleAction, LBV_Action_MDI):
    __gtype_name__ = 'LBV_ToggleAction_MDI'
    __gsignals__ = {
        'mdi-command-start': (gobject.SIGNAL_RUN_FIRST, gobject.TYPE_NONE, ()),
        'mdi-command-stop':  (gobject.SIGNAL_RUN_FIRST, gobject.TYPE_NONE, ()),
    }
    command = gobject.property(type=str, default='', nick='MDI Command')

    def _hal_init(self):
        _LBV_ToggleAction._hal_init(self)
        LBV_Action_MDI._hal_init(self)

    def on_toggled(self, w):
        if not self.get_active():
            return
        self.set_sensitive(False)
        self.emit('mdi-command-start')
        self.on_activate(w)
        gobject.timeout_add(100, self.wait_complete)

    def wait_complete(self):
        if self.labvcnc.wait_complete(0) in [-1, labvcnc.RCS_EXEC]:
            return True
        self.emit('mdi-command-stop')
        self.set_active_safe(False)
        self.set_sensitive(self.machine_on())
        return False

class LBV_Action_UnHome(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Unhome'
    axis = gobject.property(type=int, default=-1, minimum=-1, nick='Axis',
                                    blurb='Axis to unhome. -1 to unhome all')
    def _hal_init(self):
        _LBV_Action._hal_init(self)
        self.set_sensitive(False)
        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on()))
        self.gstat.connect('interp-run', lambda w: self.set_sensitive(False))

    def on_activate(self, w):
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_MANUAL)
        self.labvcnc.teleop_enable(False)
        self.labvcnc.unhome(self.axis)

def prompt_areyousure(type, message, secondary=None):
    dialog = gtk.MessageDialog(None, 0, type, gtk.BUTTONS_YES_NO, message)
    if secondary:
        dialog.format_secondary_text(secondary)
    r = dialog.run()
    dialog.destroy()
    return r == gtk.RESPONSE_YES

class LBV_Action_Home(_LBV_Action):
    __gtype_name__ = 'LBV_Action_Home'
    axis = gobject.property(type=int, default=-1, minimum=-1, nick='Axis',
                                    blurb='Axis to home. -1 to home all')
    confirm_homed = gobject.property(type=bool, default=False, nick='Confirm rehoming',
                                     blurb='Ask user if axis is already homed')
    def _hal_init(self):
        _LBV_Action._hal_init(self)
        self.set_sensitive(False)
        self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on()))
        self.gstat.connect('interp-run', lambda w: self.set_sensitive(False))

    def homed(self):
        if self.axis != -1:
            return self.stat.homed[self.axis]
        for i,h in enumerate(self.stat.homed):
            if h and self.stat.axis_mask & (1<<i):
                return True

    def on_activate(self, w):
        #if not manual_ok(): return
        ensure_mode(self.stat, self.labvcnc, labvcnc.MODE_MANUAL)
        if self.confirm_homed and self.homed():
            if not prompt_areyousure(gtk.MESSAGE_WARNING,
                            _("Axis is already homed, are you sure you want to re-home?")):
                return
        self.labvcnc.teleop_enable(False)
        self.labvcnc.home(self.axis)


class State_Sensitive_Table(gtk.Table, _LBV_ActionBase):
    __gtype_name__ = "State_Sensitive_Table"
    is_homed = gobject.property(type=bool, default=True, nick='Must Be Homed',
                                    blurb='Machine Must be homed for widgets to be sensitive to input')
    is_on = gobject.property(type=bool, default=True, nick='Must Be On',
                                    blurb='Machine Must be On for widgets to be sensitive to input')
    is_idle = gobject.property(type=bool, default=True, nick='Must Be Idle',
                                    blurb='Machine Must be Idle for widgets to be sensitive to input')

    def _hal_init(self):
        _LBV_ActionBase._hal_init(self)
        self.set_sensitive(False)
        self.gstat.connect('state-estop', lambda w: self.set_sensitive(False))
        if self.is_on:
            self.gstat.connect('state-off', lambda w: self.set_sensitive(False))
        if self.is_homed:
            self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on() and ( self.is_all_homed() or self.no_home_required() ) ))
        else:
            self.gstat.connect('interp-idle', lambda w: self.set_sensitive(self.machine_on()) )
        if self.is_idle:
            self.gstat.connect('interp-run', lambda w: self.set_sensitive(False))
        if self.is_homed:
            self.gstat.connect('all-homed', lambda w: self.set_sensitive(self.machine_on()))
