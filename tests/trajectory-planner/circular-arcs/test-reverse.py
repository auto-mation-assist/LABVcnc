"""
How to use this test script:

    This is a quick and dirty way to illustrate reverse-run behavior with existing GUI's.
    1) Launch labvcnc, set up / run a program as usual
    2) Pause the program
    3) Open a new terminal and launch this script interactively:
        python -i test-reverse.py
    4) Press "resume" in GUI to see reverse run motion (can pause and resume as with normal motion)
    5) manually type this command to run in forward direction:
        c.auto(labvcnc.AUTO_FORWARD)
    6) Resume motion in GUI with resume button

"""

import labvcnc

c = labvcnc.command()
#NOTE probably should be done already paused
c.auto(labvcnc.AUTO_PAUSE)
c.wait_complete(1)
#TODO use constant
c.auto(labvcnc.AUTO_REVERSE)
#Now play in GUI, pause / resume / etc
# Once reversing is done, pause the motion and run this command, then resume
#c.auto(labvcnc.AUTO_FORWARD)
