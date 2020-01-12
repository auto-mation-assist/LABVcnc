/*    This is a component of LabvCNC
 *    Copyright 2011, 2012, 2013 Michael Haberler <git@mah.priv.at>,
 *    Sebastian Kuzminsky <seb@highlab.com>
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
// factored out from lbvglb.h so subsystems not requiring the
// lbvglb.h defines may include them as well



#define LBV_DEBUG_CONFIG            0x00000002
#define LBV_DEBUG_VERSIONS          0x00000008
#define LBV_DEBUG_TASK_ISSUE        0x00000010
#define LBV_DEBUG_NML               0x00000040
#define LBV_DEBUG_MOTION_TIME       0x00000080
#define LBV_DEBUG_INTERP            0x00000100
#define LBV_DEBUG_RCS               0x00000200
#define LBV_DEBUG_INTERP_LIST       0x00000800
#define LBV_DEBUG_IOCONTROL         0x00001000
#define LBV_DEBUG_OWORD             0x00002000
#define LBV_DEBUG_REMAP             0x00004000
#define LBV_DEBUG_PYTHON            0x00008000
#define LBV_DEBUG_NAMEDPARAM        0x00010000
#define LBV_DEBUG_GDBONSIGNAL       0x00020000
#define LBV_DEBUG_PYTHON_TASK       0x00040000

// not interpreted by LBV.
#define LBV_DEBUG_USER1             0x10000000
#define LBV_DEBUG_USER2             0x20000000

#define LBV_DEBUG_UNCONDITIONAL     0x40000000  // always logged
#define LBV_DEBUG_ALL               0x7FFFFFFF	/* it's an int for %i to work
						 */
// debug prefix flags
#define LOG_TIME  1
#define LOG_PID   2
#define LOG_FILENAME 4 // and line
