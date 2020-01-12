#   This is a component of LabvCNC
#   Copyright 2011 Michael Haberler <git@mah.priv.at>
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
import lbvcanon
import interpreter

def failingprolog(self,*args, **words):
    lbvcanon.MESSAGE("failingprolog returning INTERP_ERROR")
    self.set_errormsg("A failed Python prolog returning INTERP_ERROR")
    return interpreter.INTERP_ERROR


