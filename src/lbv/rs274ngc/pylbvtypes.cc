/*    This is a component of LabvCNC
 *    Copyright 2013 Michael Haberler <git@mah.priv.at>
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
// Interpreter internals - Python bindings
// Michael Haberler 7/2011
//

#define BOOST_PYTHON_MAX_ARITY 9
#include <boost/python/class.hpp>
#include <boost/python/tuple.hpp>
namespace bp = boost::python;

#include "rs274ngc.hh"

static bp::object pmcartesian_str( PmCartesian &c) {
    return  bp::object("PmCartesian(x=%.4f y=%.4f z=%.4f)" %
		       bp::make_tuple(c.x,c.y,c.z));
}

static bp::object lbvpose_2_obj ( LbvPose &p) {
    return  bp::object("x=%.4f y=%.4f z=%.4f a=%.4f b=%.4f c=%.4f u=%.4f v=%.4f w=%.4f" %
		       bp::make_tuple(p.tran.x,p.tran.y,p.tran.z,
				      p.a,p.b,p.c,p.u,p.v,p.w));
}
static bp::object lbvpose_str( LbvPose &p) {
    return  bp::object("LbvPose(" + lbvpose_2_obj(p) + ")");
}

static void  set_x(LbvPose &p, double value) { p.tran.x = value; }
static void  set_y(LbvPose &p, double value) { p.tran.y = value; }
static void  set_z(LbvPose &p, double value) { p.tran.z = value; }
static double get_x(LbvPose &p) { return p.tran.x; }
static double get_y(LbvPose &p) { return p.tran.y; }
static double get_z(LbvPose &p) { return p.tran.z; }

static bp::object tool_str( CANON_TOOL_TABLE &t) {
    return  bp::object("Tool(T%d D%.4f I%.4f J%.4f Q%d offset: " %
		       bp::make_tuple(t.toolno,  t.diameter,
				      t.frontangle,t.backangle, t.orientation) +
		       lbvpose_2_obj(t.offset) + ")");
}

static void tool_zero( CANON_TOOL_TABLE &t) {
    t.toolno = -1;
    ZERO_LBV_POSE(t.offset);
    t.diameter = 0.0;
    t.frontangle = 0.0;
    t.backangle = 0.0;
    t.orientation = 0;
}
static void carte_zero( PmCartesian &c) { 
    c.x = 0.0;
    c.y = 0.0;
    c.z = 0.0;
}

static void pose_zero( LbvPose &p) { ZERO_LBV_POSE(p); }


void export_LbvTypes()
{
    using namespace boost::python;
    using namespace boost;

    // PmCartesian and LbvPose make sense to be instantiable 
    // within Python, since some canon calls take these as parameter
    class_<PmCartesian, noncopyable>("PmCartesian","LBV cartesian postition")
	.def_readwrite("x",&PmCartesian::x)
	.def_readwrite("y",&PmCartesian::y)
	.def_readwrite("z",&PmCartesian::z)
	.def("__str__", &pmcartesian_str)
	.def("zero", &carte_zero)
	;

    class_<LbvPose>("LbvPose","LBV pose")
	.def_readwrite("tran",&LbvPose::tran)
	.add_property("x", &get_x, &set_x)
	.add_property("y", &get_y, &set_y)
	.add_property("z", &get_z, &set_z)
	.def_readwrite("a",&LbvPose::a)
	.def_readwrite("b",&LbvPose::b)
	.def_readwrite("c",&LbvPose::c)
	.def_readwrite("u",&LbvPose::u)
	.def_readwrite("v",&LbvPose::v)
	.def_readwrite("w",&LbvPose::w)
	.def("__str__", &lbvpose_str)
	.def("zero", &pose_zero)
	;

    // leave CANON_TOOL_TABLE copyable/assignable because assignment is
    // used a lot when fiddling with tooltable entries
    class_<CANON_TOOL_TABLE >("CANON_TOOL_TABLE","Tool description" ,no_init)
	.def_readwrite("toolno", &CANON_TOOL_TABLE::toolno)
	.def_readwrite("offset", &CANON_TOOL_TABLE::offset)
	.def_readwrite("diameter", &CANON_TOOL_TABLE::diameter)
	.def_readwrite("frontangle", &CANON_TOOL_TABLE::frontangle)
	.def_readwrite("backangle", &CANON_TOOL_TABLE::backangle)
	.def_readwrite("orientation", &CANON_TOOL_TABLE::orientation)
	.def("__str__", &tool_str)
	.def("zero", &tool_zero)
	;
}
