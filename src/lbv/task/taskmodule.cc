/*    This is a component of LabvCNC
 *    Copyright 2011, 2012, 2014 Jeff Epler <jepler@unpythonic.net>,
 *    Michael Haberler <git@mah.priv.at>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */
// TODO: reuse interp converters

#define BOOST_PYTHON_MAX_ARITY 7
#include <boost/python/class.hpp>
#include <boost/python/def.hpp>
#include <boost/python/implicit.hpp>
#include <boost/python/module.hpp>
#include <boost/python/overloads.hpp>
#include <boost_pyenum_macros.hh>
#include <boost/python/scope.hpp>
#include "python_plugin.hh"
#include "rs274ngc.hh"
#include "interp_internal.hh"
#include "taskclass.hh"
#include "initool.hh"
#include "lbvglb.h"		// LBV_INIFILE

namespace bp = boost::python;

#include "array1.hh"

namespace pp = pyplusplus::containers::static_sized;

#include "interp_array_types.hh"  // import activeMCodes,activeGCodes,activeSettings, toolTable

#include "rcs.hh"		// NML classes, nmlErrorFormat()
#include "lbv.hh"		// LBV NML
#include "lbv_nml.hh"

extern void lbvtask_quit(int sig);
extern LBV_STAT *lbvStatus;
typedef boost::shared_ptr< LBV_STAT > lbvstatus_ptr;
extern int return_int(const char *funcname, PyObject *retval);


// man, is this ugly. I'm taking suggestions to make this better

static int handle_exception(const char *name)
{
    std::string msg = handle_pyerror();
    printf("%s(): %s\n", name, msg.c_str());
    PyErr_Clear();
    return -1;
}

#define EXPAND(method)						\
    int method() {						\
	if (bp::override f = this->get_override(#method)) {	\
	    try {						\
		return f();					\
	    }							\
	    catch( bp::error_already_set ) {			\
		return handle_exception(#method);		\
	    }							\
	}							\
	else							\
	    return  Task::method();				\
    }


#define EXPAND1(method,type,name)					\
    int method(type name) {						\
	if (bp::override f = this->get_override(#method)) {		\
	    try {							\
		return f(name);						\
	    }								\
	    catch( bp::error_already_set ) {				\
		return handle_exception(#method);			\
	    }								\
	} else								\
	    return  Task::method(name);					\
    }


#define EXPAND2(method,type,name,type2,name2)				\
    int method(type name,type2 name2) {					\
	if (bp::override f = this->get_override(#method)) {		\
	    try {							\
		return f(name,name2);					\
	    }								\
	    catch( bp::error_already_set ) {				\
		return handle_exception(#method);			\
	    }								\
	} else								\
	    return  Task::method(name,name2);				\
    }

struct TaskWrap : public Task, public bp::wrapper<Task> {

    TaskWrap() : Task() {}

    EXPAND(lbvIoInit)
    EXPAND(lbvIoHalt)
    EXPAND1(lbvIoAbort,int,reason)

    EXPAND(lbvToolStartChange)
    EXPAND(lbvAuxEstopOn)
    EXPAND(lbvAuxEstopOff)
    EXPAND(lbvCoolantMistOn)
    EXPAND(lbvCoolantMistOff)
    EXPAND(lbvCoolantFloodOn)
    EXPAND(lbvCoolantFloodOff)
    EXPAND(lbvLubeOn)
    EXPAND(lbvLubeOff)
    EXPAND1(lbvIoSetDebug,int,debug)

    EXPAND2(lbvToolPrepare,int, p, int, tool)
    EXPAND(lbvToolLoad)
    EXPAND1(lbvToolLoadToolTable, const char *, file)
    EXPAND(lbvToolUnload)
    EXPAND1(lbvToolSetNumber,int,number)

    int lbvIoPluginCall(int len,const char *msg) {
	if (bp::override f = this->get_override("lbvIoPluginCall")) {
	    try {
		// binary picklings may contain zeroes
		std::string buffer(msg,len);
		return f(len,buffer);
	    }
	    catch( bp::error_already_set ) {
		return handle_exception("lbvIoPluginCall");
	    }
	} else
	    return  Task::lbvIoPluginCall(len,msg);
    }

    int lbvToolSetOffset(int pocket, int toolno, LbvPose offset, double diameter,
			 double frontangle, double backangle, int orientation) {
	if (bp::override f = this->get_override("lbvToolSetOffset"))
	    try {
		return f(pocket,toolno,offset,diameter,frontangle,backangle,orientation);
	    }
	    catch( bp::error_already_set ) {
		return handle_exception("lbvToolSetOffset");
	    }
	else
	    return  Task::lbvToolSetOffset(pocket,toolno,offset,diameter,frontangle,backangle,orientation);
    }

    int lbvIoUpdate(LBV_IO_STAT * stat) {
	if (bp::override f = this->get_override("lbvIoUpdate"))
	    try {
		return f(); /// bug in Boost.Python, fixed in 1.44 I guess: return_int("foo",f());
	    }
	    catch( bp::error_already_set ) {
		return handle_exception("lbvIoUpdate");
	    }
	else
	    return  Task::lbvIoUpdate(stat);
    }

};

typedef pp::array_1_t< LBV_AXIS_STAT, LBVMOT_MAX_AXIS> axis_array, (*axis_w)( LBV_MOTION_STAT &m );
typedef pp::array_1_t< LBV_SPINDLE_STAT, LBVMOT_MAX_SPINDLES> spindle_array, (*spindle_w)( LBV_MOTION_STAT &m );
typedef pp::array_1_t< int, LBVMOT_MAX_DIO> synch_dio_array, (*synch_dio_w)( LBV_MOTION_STAT &m );
typedef pp::array_1_t< double, LBVMOT_MAX_AIO> analog_io_array, (*analog_io_w)( LBV_MOTION_STAT &m );

typedef pp::array_1_t< int, ACTIVE_G_CODES> active_g_codes_array, (*active_g_codes_tw)( LBV_TASK_STAT &t );
typedef pp::array_1_t< int, ACTIVE_M_CODES> active_m_codes_array, (*active_m_codes_tw)( LBV_TASK_STAT &t );
typedef pp::array_1_t< double, ACTIVE_SETTINGS> active_settings_array, (*active_settings_tw)( LBV_TASK_STAT &t );

typedef pp::array_1_t< CANON_TOOL_TABLE, CANON_POCKETS_MAX> tool_array, (*tool_w)( LBV_TOOL_STAT &t );

static  tool_array tool_wrapper ( LBV_TOOL_STAT & t) {
    return tool_array(t.toolTable);
}

static  axis_array axis_wrapper ( LBV_MOTION_STAT & m) {
    return axis_array(m.axis);
}

static  spindle_array spindle_wrapper ( LBV_MOTION_STAT & m) {
    return spindle_array(m.spindle);
}

static  synch_dio_array synch_di_wrapper ( LBV_MOTION_STAT & m) {
    return synch_dio_array(m.synch_di);
}

static  synch_dio_array synch_do_wrapper ( LBV_MOTION_STAT & m) {
    return synch_dio_array(m.synch_do);
}

static  analog_io_array analog_input_wrapper ( LBV_MOTION_STAT & m) {
    return analog_io_array(m.analog_input);
}

static  analog_io_array analog_output_wrapper ( LBV_MOTION_STAT & m) {
    return analog_io_array(m.analog_output);
}

static  active_g_codes_array activeGCodes_wrapper (  LBV_TASK_STAT & m) {
    return active_g_codes_array(m.activeGCodes);
}

static  active_m_codes_array activeMCodes_wrapper ( LBV_TASK_STAT & m) {
    return active_m_codes_array(m.activeMCodes);
}

static  active_settings_array activeSettings_wrapper ( LBV_TASK_STAT & m) {
    return active_settings_array(m.activeSettings);
}

static const char *get_file( LBV_TASK_STAT &t) { return t.file; }
static const char *get_command( LBV_TASK_STAT &t) { return t.command; }

static void operator_error(const char *message, int id = 0) {
    lbvOperatorError(id,"%s",message);
}
static void operator_text(const char *message, int id = 0) {
    lbvOperatorText(id,"%s",message);
}
static void operator_display(const char *message, int id = 0) {
    lbvOperatorDisplay(id,"%s",message);
}


#pragma GCC diagnostic push
#if defined(__GNUC__) && ((__GNUC__ > 4) || ((__GNUC__ == 4) && (__GNUC_MINOR__ >= 7)))
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
#endif
BOOST_PYTHON_FUNCTION_OVERLOADS(operator_error_overloads, operator_error, 1,2)
BOOST_PYTHON_FUNCTION_OVERLOADS(operator_text_overloads, operator_text, 1,2)
BOOST_PYTHON_FUNCTION_OVERLOADS(operator_display_overloads, operator_display, 1,2)
#pragma GCC diagnostic pop


static const char *ini_filename() { return lbv_inifile; }

BOOST_PYTHON_MODULE(lbvtask) {
    using namespace boost::python;
    using namespace boost;

    scope().attr("__doc__") =
        "Task introspection\n"
        ;

    def("lbvtask_quit", lbvtask_quit);
    def("ini_filename", ini_filename);
    // def("iniTool", iniTool);

    def("operator_error",
	operator_error,
	operator_error_overloads ( args("id"),
				   "send an error message to the operator screen with an optional message id"  ));
    def("operator_text",
	operator_text,
	operator_text_overloads ( args("id"),
				   "send a informational message to the operator screen"  ));
    def("operator_display",
	operator_display,
	operator_display_overloads ( args("id"),
				   "send a message to the operator display"  ));

    BOOST_PYENUM_(RCS_STATUS)
            .BOOST_PYENUM_VAL(RCS_EXEC)
            .BOOST_PYENUM_VAL(RCS_DONE)
            .BOOST_PYENUM_VAL(RCS_ERROR)
            ;

    BOOST_PYENUM_(LBV_TASK_MODE_ENUM)
            .BOOST_PYENUM_VAL(LBV_TASK_MODE_MANUAL)
            .BOOST_PYENUM_VAL(LBV_TASK_MODE_AUTO)
            .BOOST_PYENUM_VAL(LBV_TASK_MODE_MDI)
            ;

    BOOST_PYENUM_(LBV_TASK_STATE_ENUM)
            .BOOST_PYENUM_VAL(LBV_TASK_STATE_ESTOP)
            .BOOST_PYENUM_VAL(LBV_TASK_STATE_ESTOP_RESET)
            .BOOST_PYENUM_VAL(LBV_TASK_STATE_OFF)
            .BOOST_PYENUM_VAL(LBV_TASK_STATE_ON)
            ;

    BOOST_PYENUM_(LBV_TASK_EXEC_ENUM)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_ERROR)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_DONE)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_WAITING_FOR_MOTION)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_WAITING_FOR_MOTION_QUEUE)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_WAITING_FOR_IO)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_WAITING_FOR_MOTION_AND_IO)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_WAITING_FOR_DELAY)
            .BOOST_PYENUM_VAL(LBV_TASK_EXEC_WAITING_FOR_SYSTEM_CMD)
            ;

    BOOST_PYENUM_(LBV_TASK_INTERP_ENUM)
            .BOOST_PYENUM_VAL(LBV_TASK_INTERP_IDLE)
            .BOOST_PYENUM_VAL(LBV_TASK_INTERP_READING)
            .BOOST_PYENUM_VAL(LBV_TASK_INTERP_PAUSED)
            .BOOST_PYENUM_VAL(LBV_TASK_INTERP_WAITING)
            ;


    BOOST_PYENUM_(LBV_IO_ABORT_REASON_ENUM)
            .BOOST_PYENUM_VAL(LBV_ABORT_TASK_EXEC_ERROR)
            .BOOST_PYENUM_VAL(LBV_ABORT_AUX_ESTOP)
            .BOOST_PYENUM_VAL(LBV_ABORT_MOTION_OR_IO_RCS_ERROR)
            .BOOST_PYENUM_VAL(LBV_ABORT_TASK_STATE_OFF)
            .BOOST_PYENUM_VAL(LBV_ABORT_TASK_STATE_ESTOP_RESET)
            .BOOST_PYENUM_VAL(LBV_ABORT_TASK_STATE_ESTOP)
            .BOOST_PYENUM_VAL(LBV_ABORT_TASK_STATE_NOT_ON)
            .BOOST_PYENUM_VAL(LBV_ABORT_TASK_ABORT)
            .BOOST_PYENUM_VAL(LBV_ABORT_USER)
            ;

    class_<TaskWrap, shared_ptr<TaskWrap>, noncopyable >("Task")

	.def_readonly("use_iocontrol", &Task::use_iocontrol)
	.def_readonly("random_toolchanger", &Task::random_toolchanger)
	.def_readonly("tooltable_filename", &Task::tooltable_filename)
	;

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    class_ <LBV_TRAJ_STAT, noncopyable>("LBV_TRAJ_STAT",no_init)
	.def_readwrite("linearUnits", &LBV_TRAJ_STAT::linearUnits )
	.def_readwrite("angularUnits", &LBV_TRAJ_STAT::angularUnits )
	.def_readwrite("cycleTime", &LBV_TRAJ_STAT::cycleTime )
	.def_readwrite("axes", &LBV_TRAJ_STAT::axes )
	.def_readwrite("axis_mask", &LBV_TRAJ_STAT::axis_mask )
	.def_readwrite("mode", &LBV_TRAJ_STAT::mode )
	.def_readwrite("enabled", &LBV_TRAJ_STAT::enabled )
	.def_readwrite("inpos", &LBV_TRAJ_STAT::inpos )
	.def_readwrite("queue", &LBV_TRAJ_STAT::queue )
	.def_readwrite("activeQueue", &LBV_TRAJ_STAT::activeQueue )
	.def_readwrite("queueFull", &LBV_TRAJ_STAT::queueFull )
	.def_readwrite("id", &LBV_TRAJ_STAT::id )
	.def_readwrite("paused", &LBV_TRAJ_STAT::paused )
	.def_readwrite("scale", &LBV_TRAJ_STAT::scale )
	.def_readwrite("position", &LBV_TRAJ_STAT::position )
	.def_readwrite("actualPosition", &LBV_TRAJ_STAT::actualPosition )
	.def_readwrite("velocity", &LBV_TRAJ_STAT::velocity )
	.def_readwrite("acceleration", &LBV_TRAJ_STAT::acceleration)
	.def_readwrite("maxVelocity", &LBV_TRAJ_STAT::maxVelocity )
	.def_readwrite("maxAcceleration", &LBV_TRAJ_STAT::maxAcceleration )
	.def_readwrite("probedPosition", &LBV_TRAJ_STAT::probedPosition )
	.def_readwrite("probe_tripped", &LBV_TRAJ_STAT::probe_tripped )
	.def_readwrite("probing", &LBV_TRAJ_STAT::probing )
	.def_readwrite("probeval", &LBV_TRAJ_STAT::probeval )
	.def_readwrite("kinematics_type", &LBV_TRAJ_STAT::kinematics_type )
	.def_readwrite("motion_type", &LBV_TRAJ_STAT::motion_type )
	.def_readwrite("distance_to_go", &LBV_TRAJ_STAT::distance_to_go )
	.def_readwrite("dtg", &LBV_TRAJ_STAT::dtg )
	.def_readwrite("current_vel", &LBV_TRAJ_STAT::current_vel )
	.def_readwrite("feed_override_enabled", &LBV_TRAJ_STAT::feed_override_enabled )
	.def_readwrite("adaptive_feed_enabled", &LBV_TRAJ_STAT::adaptive_feed_enabled )
	.def_readwrite("feed_hold_enabled", &LBV_TRAJ_STAT::feed_hold_enabled )
	;
#pragma GCC diagnostic pop
    class_ <LBV_JOINT_STAT, noncopyable>("LBV_JOINT_STAT",no_init)
	.def_readwrite("units", &LBV_JOINT_STAT::units)
	.def_readwrite("backlash", &LBV_JOINT_STAT::backlash)
	.def_readwrite("minPositionLimit", &LBV_JOINT_STAT::minPositionLimit)
	.def_readwrite("maxPositionLimit" ,&LBV_JOINT_STAT::maxPositionLimit)
	.def_readwrite("maxFerror", &LBV_JOINT_STAT::maxFerror)
	.def_readwrite("minFerror", &LBV_JOINT_STAT::minFerror)
	.def_readwrite("ferrorCurrent", &LBV_JOINT_STAT::ferrorCurrent)
	.def_readwrite("ferrorHighMark", &LBV_JOINT_STAT::ferrorHighMark)
	.def_readwrite("output", &LBV_JOINT_STAT::output)
	.def_readwrite("input", &LBV_JOINT_STAT::input)
	.def_readwrite("velocity", &LBV_JOINT_STAT::velocity)
	.def_readwrite("inpos",  &LBV_JOINT_STAT::inpos)
	.def_readwrite("homing",  &LBV_JOINT_STAT::homing)
	.def_readwrite("homed",  &LBV_JOINT_STAT::homed)
	.def_readwrite("fault",  &LBV_JOINT_STAT::fault)
	.def_readwrite("enabled",  &LBV_JOINT_STAT::enabled)
	.def_readwrite("minSoftLimit",  &LBV_JOINT_STAT::minSoftLimit)
	.def_readwrite("maxSoftLimit",  &LBV_JOINT_STAT::maxSoftLimit)
	.def_readwrite("minHardLimit",  &LBV_JOINT_STAT::minHardLimit)
	.def_readwrite("maxHardLimit",  &LBV_JOINT_STAT::maxHardLimit)
	.def_readwrite("overrideLimits",  &LBV_JOINT_STAT::overrideLimits)
	;

    class_ <LBV_SPINDLE_STAT, noncopyable>("LBV_SPINDLE_STAT",no_init)
	.def_readwrite("speed", &LBV_SPINDLE_STAT::speed )
	.def_readwrite("direction", &LBV_SPINDLE_STAT::direction )
	.def_readwrite("brake", &LBV_SPINDLE_STAT::brake )
	.def_readwrite("increasing", &LBV_SPINDLE_STAT::increasing )
	.def_readwrite("enabled", &LBV_SPINDLE_STAT::enabled )
	.def_readwrite("spindle_override_enabled", &LBV_SPINDLE_STAT::spindle_override_enabled )
	.def_readwrite("spindle_scale", &LBV_SPINDLE_STAT::spindle_scale )
	.def_readwrite("spindle_orient_state", &LBV_SPINDLE_STAT::orient_state )
	.def_readwrite("spindle_orient_fault", &LBV_SPINDLE_STAT::orient_fault )
	;

    class_ <LBV_COOLANT_STAT , noncopyable>("LBV_COOLANT_STAT ",no_init)
	.def_readwrite("mist", &LBV_COOLANT_STAT::mist )
	.def_readwrite("flood", &LBV_COOLANT_STAT::flood )
	;

    class_ <LBV_LUBE_STAT, noncopyable>("LBV_LUBE_STAT",no_init)
	.def_readwrite("on", &LBV_LUBE_STAT::on )
	.def_readwrite("level", &LBV_LUBE_STAT::level )
	;

    class_ <LBV_MOTION_STAT, noncopyable>("LBV_MOTION_STAT",no_init)
	.def_readwrite("traj", &LBV_MOTION_STAT::traj)
	.add_property( "axis",
		       bp::make_function( axis_w(&axis_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "spindle",
			   bp::make_function( spindle_w(&spindle_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "synch_di",
		       bp::make_function( synch_dio_w(&synch_di_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "synch_do",
		       bp::make_function( synch_dio_w(&synch_do_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "analog_input",
		       bp::make_function( analog_io_w(&analog_input_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "analog_output",
		       bp::make_function( analog_io_w(&analog_output_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	;


    class_ <LBV_TASK_STAT, noncopyable>("LBV_TASK_STAT",no_init)
	.def_readwrite("mode",  &LBV_TASK_STAT::mode)
	.def_readwrite("state",  &LBV_TASK_STAT::state)
	.def_readwrite("execState",  &LBV_TASK_STAT::execState)
	.def_readwrite("interpState",  &LBV_TASK_STAT::interpState)
	.def_readwrite("motionLine", &LBV_TASK_STAT::motionLine)
	.def_readwrite("currentLine", &LBV_TASK_STAT::currentLine)
	.def_readwrite("readLine", &LBV_TASK_STAT::readLine)
	.def_readwrite("optional_stop_state", &LBV_TASK_STAT::optional_stop_state)
	.def_readwrite("block_delete_state", &LBV_TASK_STAT::block_delete_state)
	.def_readwrite("input_timeout", &LBV_TASK_STAT::input_timeout)

	//  read-only
	.add_property("file",  &get_file)
	.add_property("command",   &get_command)

	.def_readwrite("g5x_offset", &LBV_TASK_STAT::g5x_offset)
	.def_readwrite("g5x_index", &LBV_TASK_STAT::g5x_index)
	.def_readwrite("g92_offset", &LBV_TASK_STAT::g92_offset)
	.def_readwrite("rotation_xy", &LBV_TASK_STAT::rotation_xy)
	.def_readwrite("toolOffset", &LBV_TASK_STAT::toolOffset)
	.add_property( "activeGCodes",
		       bp::make_function( active_g_codes_tw(&activeGCodes_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "activeMCodes",
		       bp::make_function( active_m_codes_tw(&activeMCodes_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.add_property( "activeSettings",
		       bp::make_function( active_settings_tw(&activeSettings_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	.def_readwrite("programUnits", &LBV_TASK_STAT::programUnits)
	.def_readwrite("interpreter_errcode", &LBV_TASK_STAT::interpreter_errcode)
	.def_readwrite("task_paused", &LBV_TASK_STAT::task_paused)
	.def_readwrite("delayLeft", &LBV_TASK_STAT::delayLeft)
	;

    class_ <LBV_TOOL_STAT, noncopyable>("LBV_TOOL_STAT",no_init)
	.def_readwrite("pocketPrepped", &LBV_TOOL_STAT::pocketPrepped )
	.def_readwrite("toolInSpindle", &LBV_TOOL_STAT::toolInSpindle )
	.add_property( "toolTable",
		       bp::make_function( tool_w(&tool_wrapper),
					  bp::with_custodian_and_ward_postcall< 0, 1 >()))
	;

    class_ <LBV_AUX_STAT, noncopyable>("LBV_AUX_STAT",no_init)
	.def_readwrite("estop", &LBV_AUX_STAT::estop)
	;

    class_ <LBV_IO_STAT, noncopyable>("LBV_IO_STAT",no_init)
	.def_readwrite("cycleTime", &LBV_IO_STAT::cycleTime )
	.def_readwrite("debug", &LBV_IO_STAT::debug )
	.def_readwrite("reason", &LBV_IO_STAT::reason )
	.def_readwrite("fault", &LBV_IO_STAT::fault )
	.def_readwrite("tool", &LBV_IO_STAT::tool)
	.def_readwrite("aux", &LBV_IO_STAT::aux)
	.def_readwrite("coolant", &LBV_IO_STAT::coolant)
	.def_readwrite("lube", &LBV_IO_STAT::lube)
	.def_readwrite("status", &LBV_IO_STAT::status)
	;


    class_ <LBV_STAT, lbvstatus_ptr, noncopyable>("LBV_STAT",no_init)
	.def_readwrite("task", &LBV_STAT::task)
	.def_readwrite("motion", &LBV_STAT::motion)
	.def_readwrite("io", &LBV_STAT::io)
	.def_readwrite("debug", &LBV_STAT::debug)

	;



    // this assumes that at module init time lbvStatus is valid (non-NULL)
    scope().attr("lbvstat") = lbvstatus_ptr(lbvStatus);

    implicitly_convertible<LBV_TASK_STATE_ENUM, int>();

    pp::register_array_1< double, LBVMOT_MAX_AIO> ("AnalogIoArray");
    pp::register_array_1< int, LBVMOT_MAX_DIO> ("DigitalIoArray");
    pp::register_array_1< LBV_AXIS_STAT,LBVMOT_MAX_AXIS,
	bp::return_internal_reference< 1, bp::default_call_policies > > ("AxisArray");
}


