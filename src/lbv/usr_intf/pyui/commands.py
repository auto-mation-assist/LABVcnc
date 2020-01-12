import labvcnc
import os,sys
# path to the configuration the user requested
# used to see if the is local handler files to use
try:
    CONFIGPATH = os.environ['CONFIG_DIR']
    CONFIGDIR = os.path.join(CONFIGPATH, 'panelui_handler.py')
except:
    print '**** PANEL COMMAND: no panelui_handlers.py file in config directory'
    CONFIGPATH = os.path.expanduser("~")
    CONFIGDIR = os.path.join(CONFIGPATH, 'panelui_handler.py')

# constants
JOGJOINT  = 1
JOGTELEOP = 0

inifile = labvcnc.ini(os.environ['INI_FILE_NAME'])
trajcoordinates = inifile.find("TRAJ", "COORDINATES").lower().replace(" ","")
jointcount = int(inifile.find("KINS","JOINTS"))

DBG_state = 0
def DBG(str):
    if DBG_state > 0:
        print str

# Loads user commands from a file named 'panelui_handler.py' from the config
def load_handlers(usermod,halcomp,builder,commands,master):
    hdl_func = 'get_handlers'
    mod = object = None
    def add_handler(method, f):
        if method in handlers:
            handlers[method].append(f)
        else:
            handlers[method] = [f]
    handlers = {}
    for u in usermod:
        (directory,filename) = os.path.split(u)
        (basename,extension) = os.path.splitext(filename)
        if directory == '':
            directory = '.'
        if directory not in sys.path:
            sys.path.insert(0,directory)
            DBG( 'panelui: adding import dir %s' % directory)
        try:
            mod = __import__(basename)
        except ImportError,msg:
            print ("panelui: module '%s' skipped - import error: %s" %(basename,msg))
	    continue
        DBG( "panelui: module '%s' imported OK" % mod.__name__)
        try:
            # look for 'get_handlers' function
            h = getattr(mod,hdl_func,None)
            if h and callable(h):
                DBG("panelui: module '%s' : '%s' function found" % (mod.__name__,hdl_func))
                objlist = h(halcomp,builder,commands,master)
            else:
                # the module has no get_handlers() callable.
                # in this case we permit any callable except class Objects in the module to register as handler
                DBG("panelui: module '%s': no '%s' function - registering only functions as callbacks" % (mod.__name__,hdl_func))
                objlist =  [mod]
            # extract callback candidates
            for object in objlist:
                #DBG("Registering handlers in module %s object %s" % (mod.__name__, object))
                if isinstance(object, dict):
                    methods = dict.items()
                else:
                    methods = map(lambda n: (n, getattr(object, n, None)), dir(object))
                for method,f in methods:
                    if method.startswith('_'):
                        continue
                    if callable(f):
                        DBG("panelui: Register callback '%s' in %s" % (method, basename))
                        add_handler(method, f)
        except Exception, e:
            print ("**** PANELUI ERROR: trouble looking for handlers in '%s': %s" %(basename, e))

    # Wrap lists in Trampoline, unwrap single functions
    for n,v in list(handlers.items()):
        if len(v) == 1:
            handlers[n] = v[0]
        else:
            handlers[n] = Trampoline(v)

    return handlers,mod,object

# trampoline and load_handlers are used for custom keyboard commands
class Trampoline(object):
    def __init__(self,methods):
        self.methods = methods

    def __call__(self, *a, **kw):
        for m in self.methods:
            m(*a, **kw)

# labvcnc commands
class CNC_COMMANDS():
        def __init__(self, master):
            global DBG_state
            DBG_state = master._dbg
            self.lbv = labvcnc
            self.lbvstat = labvcnc.stat()
            self.lbvcommand = labvcnc.command()
            self.return_to_mode = -1 # if not -1 return to the mode specified
            self.sb = 0;
            self.jog_velocity = 100.0/60.0
            self.angular_jog_velocity = 3600/60
            self._mdi = 0
            self.isjogging = [0,0,0,0,0,0,0,0,0]
            self.restart_line_number = self.restart_reset_line = 0
            try:
                handlers,self.handler_module,self.handler_instance = \
                load_handlers([CONFIGDIR], self.lbvstat, self.lbvcommand,self, master)
            except Exception, e:
                print e

        def mdi_active(self, wname, m):
            self._mdi = m

        def mist_on(self, wname, b):
            self.lbvcommand.mist(1)

        def mist_off(self, wname, b):
            self.lbvcommand.mist(0)

        def flood_on(self, wname, b):
            self.lbvcommand.flood(1)

        def flood_off(self, wname, b):
            self.lbvcommand.flood(0)

        def estop(self, wname, b):
            self.lbvcommand.state(self.lbv.STATE_ESTOP)

        def estop_reset(self, wname, b):
            self.lbvcommand.state(self.lbv.STATE_ESTOP_RESET)

        def machine_off(self, wname, b):
            self.lbvcommand.state(self.lbv.STATE_OFF)

        def machine_on(self, wname, b):
            self.lbvcommand.state(self.lbv.STATE_ON)

        def home_all(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.home(-1)

        def unhome_all(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.unhome(-1)

        def home_selected(self, wname, joint):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.home(int(joint))

        def unhome_selected(self, wname, joint):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.unhome(int(joint))

        def jogging(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)

        def override_limits(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.override_limits()

        def spindle_forward_adjust(self, wname, rpm=100):
            if self.get_mode() == self.lbv.MODE_MDI:
                self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            speed = self.is_spindle_running()
            if  speed == 0:
                self.lbvcommand.spindle(1,float(rpm));
            elif speed > 0:
                self.lbvcommand.spindle(self.lbv.SPINDLE_INCREASE)
            else:
                self.lbvcommand.spindle(self.lbv.SPINDLE_DECREASE)

        def spindle_forward(self, wname, rpm=100):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            speed = self.is_spindle_running()
            if  speed == 0:
                self.lbvcommand.spindle(1,float(rpm));

        def spindle_stop(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.spindle(0);

        def spindle_reverse(self, wname, rpm=100):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            speed = self.is_spindle_running()
            if  speed == 0:
                self.lbvcommand.spindle(-1,float(rpm));

        def spindle_reverse_adjust(self, wname, rpm=100):
            if self.get_mode() == self.lbv.MODE_MDI:
                self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            speed = self.is_spindle_running()
            if  speed == 0:
                self.lbvcommand.spindle(-1,float(rpm));
            elif speed < 0:
                self.lbvcommand.spindle(self.lbv.SPINDLE_INCREASE)
            else:
                self.lbvcommand.spindle(self.lbv.SPINDLE_DECREASE)

        def spindle_faster(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.spindle(0,self.lbv.SPINDLE_INCREASE)

        def spindle_slower(self, wname, b):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.spindle(0,self.lbv.SPINDLE_DECREASE)

        def set_linear_jog_velocity(self, wname, cmd):
            velocity = float(cmd)
            if velocity is not None:
                rate = self.jog_velocity = velocity / 60.0
                for axisnum in (0,1,2,6,7,8):
                    if self.isjogging[axisnum]:
                        jjogmode,j_or_a = self.get_jog_info(axisnum)
                        self.lbvcommand.jog(self.lbv.JOG_CONTINUOUS, jjogmode, j_or_a, self.isjogging[i] * rate)

        def set_angular_jog_velocity(self, wname, cmd):
            angular = float(cmd)
            if velocity is not None:
                rate = self.angular_jog_velocity = angular / 60.0
                for axisnum in (3,4,5):
                    if self.isjogging[axisnum]:
                        jjogmode,j_or_a = self.get_jog_info(axisnum)
                        self.lbvcommand.jog(self.lbv.JOG_CONTINUOUS, jjogmode, j_or_a, self.isjogging[i] * rate)

        def continuous_jog(self, wname, cmd):
            axisnum = int(cmd[0])
            jjogmode,j_or_a = self.get_jog_info(axisnum)
            direction = int(cmd[1])
            if direction == 0:
                self.isjogging[axisnum] = 0
                self.lbvcommand.jog(self.lbv.JOG_STOP, jjogmode, j_or_a)
            else:
                if axisnum in (3,4,5):
                    rate = self.angular_jog_velocity
                else:
                    rate = self.jog_velocity
                self.isjogging[axisnum] = direction
                self.lbvcommand.jog(self.lbv.JOG_CONTINUOUS, jjogmode, j_or_a, direction * rate)

        def incremental_jog(self, wname, cmd):
            axisnum = int(cmd[0])
            jjogmode,j_or_a = self.get_jog_info(axisnum)
            direction = int(cmd[1])
            distance = float(cmd[2])
            self.isjogging[axisnum] = direction
            if axisnum in (3,4,5):
                rate = self.angular_jog_velocity
            else:
                rate = self.jog_velocity
            self.lbvcommand.jog(self.lbv.JOG_INCREMENT, jjogmode, axisnum, direction * rate, distance)
            self.isjogging[axisnum] = 0

        def quill_up(self, wname, cmd):
            self.lbvcommand.mode(self.lbv.MODE_MANUAL)
            self.lbvcommand.wait_complete()
            self.mdi('G53 G0 Z %f'% float(cmd))

        def feed_hold(self, wname, cmd):
            self.lbvcommand.set_feed_hold(int(cmd))

        def feed_override(self, wname, f):
            self.lbvcommand.feedrate(f)

        def rapid_override(self, wname, f):
            self.lbvcommand.rapidrate(f)

        def spindle_override(self, wname, s):
            self.lbvcommand.spindleoverride(0,s)

        def max_velocity(self, wname, m):
            self.lbvcommand.maxvel(m)

        def reload_tooltable(self, wname, b):
            self.lbvcommand.load_tool_table()

        def optional_stop(self, wname, cmd):
            self.lbvcommand.set_optional_stop(int(cmd))

        def block_delete(self, wname, cmd):
            self.lbvcommand.set_block_delete(int(cmd))

        def abort(self, wname, cmd=None):
            self.lbvcommand.abort()

        def pause(self, wname, cmd=None):
            self.lbvcommand.auto(self.lbv.AUTO_PAUSE)

        def resume(self, wname, cmd=None):
            self.lbvcommand.auto(self.lbv.AUTO_RESUME)

        def single_block(self, wname, s):
            self.sb = s
            self.lbvstat.poll()
            if self.lbvstat.queue > 0 or self.lbvstat.paused:
                # program or mdi is running
                if s:
                    self.lbvcommand.auto(self.lbv.AUTO_PAUSE)
                else:
                    self.lbvcommand.auto(self.lbv.AUTO_RESUME)

        # make sure labvcnc is in AUTO mode
        # if Labvcnc is paused then pushing cycle start will step the program
        # else the program starts from restart_line_number
        # after restarting it resets the restart_line_number to 0.
        # You must explicitily set a different restart line each time
        def smart_cycle_start(self, wname, cmd=None):
            self.lbvstat.poll()
            if self.lbvstat.task_mode != self.lbv.MODE_AUTO:
                self.lbvcommand.mode(self.lbv.MODE_AUTO)
                self.lbvcommand.wait_complete()
            self.lbvstat.poll()
            if self.lbvstat.paused:
                self.lbvcommand.auto(self.lbv.AUTO_STEP)
                return
            if self.lbvstat.interp_state == self.lbv.INTERP_IDLE:
                print self.restart_line_number
                self.lbvcommand.auto(self.lbv.AUTO_RUN, self.restart_line_number)
            self.restart_line_number = self.restart_reset_line

        # This restarts the program at the line specified directly (without cyscle start push)
        def re_start(self, wname, line):
            self.lbvcommand.mode(self.lbv.MODE_AUTO)
            self.lbvcommand.wait_complete()
            self.lbvcommand.auto(self.lbv.AUTO_RUN, line)
            self.restart_line_number = self.restart_reset_line

        # checks if ready for commands
        # calls MDI commands and when idle, periodic() will return to the mode it was in
        def mdi_and_return(self, wname, cmd):
            if self.ok_for_mdi():
                self.return_to_mode = self.get_mode() # for periodic()
                self.set_mdi_mode()
                if isinstance(cmd,list):
                    for i in cmd:
                        print str(i)
                        self.lbvcommand.mdi(str(i))
                else:
                    self.lbvcommand.mdi(str(cmd))

        # call MDI commands, set mode if needed
        def mdi(self, wname, cmd):
            self.set_mdi_mode()
            if isinstance(cmd,list):
                for i in cmd:
                    print str(i)
                    self.lbvcommand.mdi(str(i))
            else:
                self.lbvcommand.mdi(str(cmd))

        # set the restart line, you can the either restart directly
        # or restart on the cycle start button push
        # see above.
        # reset option allows one to change the default restart after it next restarts
        # eg while a restart dialog is open, always restart at the line it says
        # when the dialog close change the  line and reset both to zero
        def set_restart_line (self,  wname, line,reset=0):
            self.restart_line_number = line
            self.restart_reset_line = reset

        def set_manual_mode(self):
            self.lbvstat.poll()
            if self.lbvstat.task_mode != self.lbv.MODE_MANUAL:
                self.lbvcommand.mode(self.lbv.MODE_MANUAL)
                self.lbvcommand.wait_complete()

        def set_mdi_mode(self):
            self.lbvstat.poll()
            if self.lbvstat.task_mode != self.lbv.MODE_MDI:
                self.lbvcommand.mode(self.lbv.MODE_MDI)
                self.lbvcommand.wait_complete()

        def set_auto_mode(self):
            self.lbvstat.poll()
            if self.lbvstat.task_mode != self.lbv.MODE_AUTO:
                self.lbvcommand.mode(self.lbv.MODE_AUTO)
                self.lbvcommand.wait_complete()

        def get_mode(self):
            self.lbvstat.poll()
            return self.lbvstat.task_mode

        def ok_for_mdi(self):
            self.lbvstat.poll()
            s = self.lbvstat
            return not s.estop and s.enabled and s.homed and \
                (s.interp_state == self.lbv.INTERP_IDLE)

        def is_spindle_running(self):
            self.lbvstat.poll()
            s = self.lbvstat
            if s.spindle[0]['enabled']:
                return s.spindle[0]['speed']
            else:
                return 0

        def periodic(self):
            # return mode back to preset variable, when idle
            if self.return_to_mode > -1:
                self.lbvstat.poll()
                if self.lbvstat.interp_state == self.lbv.INTERP_IDLE:
                    self.lbvcommand.mode(self.return_to_mode)
                    self.return_to_mode = -1

        def __getitem__(self, item):
            return getattr(self, item)
        def __setitem__(self, item, value):
            return setattr(self, item, value)

        def get_jjogmode(self):
            self.lbvstat.poll()
            if self.lbvstat.motion_mode == labvcnc.TRAJ_MODE_FREE:
                return JOGJOINT
            if self.lbvstat.motion_mode == labvcnc.TRAJ_MODE_TELEOP:
                return JOGTELEOP
            print "commands.py: unexpected motion_mode",self.lbvstat.motion_mode
            return JOGTELEOP

        def jnum_for_axisnum(self,axisnum):
            if self.lbvstat.kinematics_type != labvcnc.KINEMATICS_IDENTITY:
                print ("\n%s:\n  Joint jogging not supported for"
                       "non-identity kinematics"%__file__)
                return -1 # lbvJogCont() et al reject neg joint/axis no.s
            jnum = trajcoordinates.index( "xyzabcuvw"[axisnum] )
            if jnum > jointcount:
                print ("\n%s:\n  Computed joint number=%d for axisnum=%d "
                       "exceeds jointcount=%d with trajcoordinates=%s"
                       %(__file__,jnum,axisnum,jointcount,trajcoordinates))
                # Note: primary gui should protect for this misconfiguration
                # decline to jog
                return -1 # lbvJogCont() et al reject neg joint/axis no.s
            return jnum

        def get_jog_info (self,axisnum):
            jjogmode = self.get_jjogmode()
            j_or_a = axisnum
            if jjogmode == JOGJOINT: j_or_a = self.jnum_for_axisnum(axisnum)
            return jjogmode,j_or_a

