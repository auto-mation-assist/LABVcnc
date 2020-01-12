#!/bin/sh
# the next line restarts using lbvsh \
exec $LABVCNC_LBVSH "$0" "$@"

###############################################################
# Description:  lbvdebug.tcl
#               This file sets debug levesl from tklbv.
#
#  Derived from a work by Fred Proctor & Will Shackleford
#  Author: 
#  License: GPL Version 2
#
#  Copyright (c) 2005-2009 All rights reserved.
###############################################################
#
# Sets value of LBV_DEBUG, so you can turn on/off what you want dumped
# Needs lbvsh to run-- this is sourced by tklbv, but it can be run
# standalone. Make sure directory containing lbvsh is in your path,
# or edit the exec line above with the path to lbvsh.
#
###############################################################

source [file join [file dirname [info script]] .. labvcnc.tcl]
eval lbv_init $argv

# check if any lbv commands exist, and quit if not
if {! [string length [info commands lbv_plat]]} {
    error "lbvdebug needs to run from \"lbvsh\""
}

proc distributeDebug {} {
    global debug_invalid
    global debug_config
    global debug_defaults
    global debug_versions
    global debug_task_issue
    global debug_io_points
    global debug_nml
    global debug_motion_time
    global debug_interp
    global debug_rcs
    global debug_traj
    global debug_interp_list

    # get the debug level from the LBV
    set lbv_debug_level [lbv_debug]

    # set the initial check boxes to correspond to current debug settings

    set debug_invalid [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_config [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_defaults [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_versions [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_task_issue [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_io_points [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_nml [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_motion_time [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_interp [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_rcs [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_traj [expr $lbv_debug_level % 2]

    set lbv_debug_level [expr $lbv_debug_level >> 1]
    set debug_interp_list [expr $lbv_debug_level % 2]
}

proc collectDebug {} {
    global debug_invalid
    global debug_config
    global debug_defaults
    global debug_versions
    global debug_task_issue
    global debug_io_points
    global debug_nml
    global debug_motion_time
    global debug_interp
    global debug_rcs
    global debug_traj
    global debug_interp_list

    set lbv_debug_level $debug_interp_list

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_traj]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_rcs]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_interp]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_motion_time]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_nml]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_io_points]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_task_issue]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_versions]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_defaults]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_config]

    set lbv_debug_level [expr $lbv_debug_level << 1]
    set lbv_debug_level [expr $lbv_debug_level + $debug_invalid]

    lbv_debug $lbv_debug_level
}

proc popdownDebug {w} {
    destroy $w
    if {$w == "."} {
        exit
    }
}

proc popupDebug {{w .debugwindow}} {
    global debug_invalid
    global debug_config
    global debug_defaults
    global debug_versions
    global debug_task_issue
    global debug_io_points
    global debug_nml
    global debug_motion_time
    global debug_interp
    global debug_rcs
    global debug_traj
    global debug_interp_list

    # trap "." as window name, and erase it, so that widgets
    # don't begin with two dots. We'll refer to the passed-in
    # name "w" here; in code following this setup we'll use the
    # modified name "lw"
    if {$w == "."} {
	# we're running standalone
        set lw ""
        wm title $w [msgcat::mc "LabvCNC Debug"]
    } else {
	# we were sourced into tklbv, and run as a popup
        set lw $w
        if {[winfo exists $w]} {
            wm deiconify $w
            raise $w
            focus $w
            return
        }
        toplevel $w
        wm title $w [msgcat::mc "LabvCNC Debug"]
    }

    # use "lw" as name of top level from now on

    # set the debug variables so that they're initially checked properly
    distributeDebug

    checkbutton $lw.b1 -text [msgcat::mc "Invalid INI file entries"] -variable debug_invalid -relief flat
    checkbutton $lw.b2 -text [msgcat::mc "Configuration information"] -variable debug_config -relief flat
    checkbutton $lw.b3 -text [msgcat::mc "Use of defaults"] -variable debug_defaults -relief flat
    checkbutton $lw.b4 -text [msgcat::mc "Version information"] -variable debug_versions -relief flat
    checkbutton $lw.b5 -text [msgcat::mc "Command issuing"] -variable debug_task_issue -relief flat
    checkbutton $lw.b6 -text [msgcat::mc "IO points"] -variable debug_io_points -relief flat
    checkbutton $lw.b7 -text [msgcat::mc "NML"] -variable debug_nml -relief flat
    checkbutton $lw.b8 -text [msgcat::mc "Motion time"] -variable debug_motion_time -relief flat
    checkbutton $lw.b9 -text [msgcat::mc "Interpreter"] -variable debug_interp -relief flat
    checkbutton $lw.b10 -text [msgcat::mc "RCS"] -variable debug_rcs -relief flat
    checkbutton $lw.b11 -text [msgcat::mc "Trajectory level"] -variable debug_traj -relief flat
    checkbutton $lw.b12 -text [msgcat::mc "Interpreter list"] -variable debug_interp_list -relief flat

    pack $lw.b1 $lw.b2 $lw.b3 $lw.b4 $lw.b5 $lw.b6 $lw.b7 $lw.b8 $lw.b9 $lw.b10 $lw.b11 $lw.b12 -side top -pady 2 -anchor w

    frame $lw.buttons
    button $lw.buttons.ok -text [msgcat::mc "OK"] -default active -command "collectDebug; popdownDebug $w"
    button $lw.buttons.cancel -text [msgcat::mc "Cancel"] -command "popdownDebug $w"
    pack $lw.buttons -side bottom -fill x -pady 2m
    pack $lw.buttons.ok $lw.buttons.cancel -side left -expand 1
    bind $w <Return> "lbv_debug [collectDebug] ; popdownDebug $w"
}

# if we're not running inside tklbv, then pop us up in root window
if {! [info exists tklbv]} {
    popupDebug .
}
