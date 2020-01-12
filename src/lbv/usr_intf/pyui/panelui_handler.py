import hal
# standard handler call - This will always be required
def get_handlers(labvcnc_stat, linucnc_cmd, commands, master):
     return [HandlerClass(labvcnc_stat, linucnc_cmd, commands, master)]

# Also required - handler class
class HandlerClass:

    # This will be pretty standard to gain access to everything
    # labvcnc_stat: is the python status instance of labvcnc
    # labvcnc_cmd: is the python command instance of labvcnc
    # commands: is the command instance so one can call the internal routines
    # master: give access to the master functions/data

    def __init__(self, labvcnc_stat, labvcnc_cmd, commands, master):
        self.parent = commands
        self.current_mode = 0

    # command functions are expected to have this layout:
    # def some_name(self, widget_instance, arguments from widget):
    # widget_instance gives access to the calling widget's function/data
    # arguments can be a list of arguments, a single argument, or None
    # depending on what was given in the confuration file.
    def hello_world(self, wname, m):
        # print to terminal so we know it worked
        print '\nHello world\n'
        # print the argument(s)
        print m
        # Print the calling widgets internal metadata (from config file)
        print wname.metadata
        # call a mdi command to print a msg in labvcnc
        # parent commands expect a widget_instance - None is substituted
        self.parent.mdi(None,'(MSG, Hello Labvcnc World!)')

    def cycle_mode(self, wname, m):
        if self.current_mode == 0:
            self.current_mode = 1
            self.parent.set_mdi_mode()
        elif self.current_mode == 1:
            self.current_mode = 2
            self.parent.set_auto_mode()
        else:
            self.current_mode = 0
            self.parent.set_manual_mode()
        print self.current_mode

    # Boiler code, often required
    def __getitem__(self, item):
        return getattr(self, item)
    def __setitem__(self, item, value):
        return setattr(self, item, value)
