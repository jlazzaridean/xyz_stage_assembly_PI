from warnings import warn
import ctypes as C
import numpy as np

# TODO: set software limits here and in ASPCii for objective safe
# movement once the stage is mounted

# This DLL file should be in the same folder as this python script. I'm
# not sure if CDLL, WinDLL, or OleDLL is more appropriate because the
# docs don't specify, but CDLL seems to be working.
dll = C.CDLL('ACSCL_x64.dll')

class ACSC_HistoryBuffer(C.Structure):
    """A structure to receive unsolicited messages from the stage."""
    _fields_ = [
        ("Max", C.c_int32), # buffer size
        ("Cur", C.c_int32), # number of bytes currently in the buffer
        ("Ring", C.c_int32), # circular index in the buffer
        ("Buf", C.c_char_p)] # pointer to the buffer

class ACSC_WaitBlock(C.Structure):
    """A structure used by the ACS Motion Control API to allow
    asynchronous function calls.
    """
    _fields_ = [
        ("Event", C.c_void_p), # not used, according to docs
        ("Ret", C.c_int32)] # the completion of a task

class PI_Custom_XYZ_Stage:
    """This code is intended to control a Physik Instrumente custom XYZ
    stage assembly, with a V-551.7D X axis (0), a V-555.1D Y axis (1),
    and a V308.753030 Z axis (2). They are controlled by an A814
    controller that is accessed by the ACS Motion Control C API.

    This class allows initialization of the stage and contains basic
    getter/setter functionality for position, velocity, and
    acceleration. The position setter functions (i.e. "move") are more
    sophisticated, with safeguards to ensure reasonable handling of
    sequential commands and asychronous movement requests.

    Because of these asynchronous calls, it is possible to enter an
    error state and not realize, as the program will not interrupt you
    when the stage throws an error. So far, the stage seems
    "well-behaved" when it errors out (i.e. it just disables the axis),
    but the error often doesn't come to light until the next function
    call. If it became important to discover errors from asynchronous
    movement calls more promptly, one could have the wrapper code
    controlling the microscope as a whole periodically check the stage
    error logs.

    This device communicates over an ethernet connection with a static
    IP address. To make it available to a host PC, you can configure
    your local LAN settings (IPv4 properties) to interact locally with
    the stage (disable DHCP, set subnet mask to 255.255.255.0, set IP to
    e.g. 10.0.0.10). These settings will disable internet access on that
    ethernet connection, so you'll need a wireless network adapter or
    another network card. Alternatively, you could get a network switch
    adn connect the A814 controller to the network and give it a static
    IP address (although that's not what we did in this case).
    """
    def __init__(self,
                 ip_address='10.0.0.100', 
                 port=701, # ACSC_SOCKET_STREAM_PORT,
                 verbose=True,
                 very_verbose=False):
        """Opens an ethernet connection to a custom Physik Instrumente
        XYZ stage. It then homes the axes and sets internal variables
        for position, velocity, and acceleration, as well as their
        respective limits.
        """
        self.handle = dll.open_ethernet_comm(
            C.c_char_p(bytes(ip_address, encoding='ascii')),
            C.c_int32(port))
        # Note: The above function doesn't return an error even if
        # communication isn't established (i.e. IP address is wrong).
        # The serial number call below is included to check this case.
        # If we didn't establish communication with the device, it will
        # fail.
        self.verbose = verbose
        self.very_verbose = very_verbose
        serial_number = C.c_char_p(b'*'*10)
        bytes_received = C.c_int32()
        wait_block = C.POINTER(ACSC_WaitBlock)()
        ans = dll.acsc_GetSerialNumber(
            self.handle,
            serial_number,
            10, # size of the buffer serial_number
            bytes_received,
            wait_block)
        self.serial_number = serial_number.value.decode('ascii')
        print('Opened PI XYZ Stage, serial number:', self.serial_number)
        # Clear all previous faults on axes
        if self.very_verbose: print('Clearing any XYZ axis faults...')
        for axis in [0, 1, 2]:
            wait = C.POINTER(ACSC_WaitBlock)()
            dll.clear_faults(self.handle, C.c_int32(axis), wait)
        # Open a message buffer to receive unsolicited messages)
        self.message_buffer = dll.open_message_buffer(
            self.handle,
            C.c_int32(2000)) # buffer size in bytes
        # Let's check the homing status & see if we are homed already
        all_ready = True
        for axis in [0, 1, 2]:
            ready = self._get_is_motor_homed_and_enabled(axis)
            if not ready:
                all_ready = False
        if not all_ready:
            warn('At least one axis on PI XYZ is not homed and enabled.')        
        # Let's put in some positional limits based on specs
        self.x_min = 0 # minimum x position in mm
        self.x_max = 230 # maximum x position in mm
        self.y_min = 40 # minimum y position in mm - avoid collision
        self.y_max = 130 # maximum y position in mm
        self.z_min = 0 # minimum z position in mm
        self.z_max = 7 # maximum z position in mm
        # Let's add in limits on velocity and acceleration too
        # These are not inherent to the stage but are based on what
        # seems reasonable for moving around microscope samples.
        self.vx_min, self.vy_min, self.vz_min = 0, 0, 0
        self.vx_max, self.vy_max, self.vz_max = 100, 100, 50
        self.ax_min, self.ay_min, self.az_min = 0, 0, 0
        self.ax_max, self.ay_max, self.az_max = 5000, 5000, 5000

        # Read position, velocity, and acceleration settings
        self.x, self.y, self.z = self.get_position()
        self.vx, self.vy, self.vz = self.get_velocity()
        self.ax, self.ay, self.az = self.get_acceleration()
        if self.verbose: print('PI XYZ stage initialized.')
        # TODO: warnings about axis homing, check whether homed
        return None

    def home_all_axes(self):
        """Runs the homing routine in buffer 2 for all three axes.
        Buffer 2 homes the axes in a safe order (home Z and move it to
        highest range, then home X and move it to center, then home Y
        and move it to center.)

        This function is a mandatory blocking call, as trying to do
        other things before the stage has homed is a dangerous
        proposition.

        """
        self.finish_moving()
        if self.verbose: print('PI XYZ stage homing all axes.')
        # Home the Z axis first so we can keep it out of the way
        wait_block = C.POINTER(ACSC_WaitBlock)()
        dll.run_buffer(
            self.handle,
            C.c_int32(2), # buffer 2 is the homing routine
            C.c_char_p(bytes('homeA', encoding='ascii')),
            wait_block)
        self.finish_program(2) # block before we do the next move
        # now center up Z so we aren't so far off
        z_center = (self.z_max - self.z_min) / 2
        self.move_absolute(z=z_center, blocking=True)
        return None

    def home_single_axis(self, axis, blocking=True):
        """Runs the preset homing program in buffer 1 for a single axis
        only. This will move the X and Z axes through their full travel
        range and will move the Y axis to its negative limit. Other
        function calls issued while the homing program is running may
        return errors, so running the homing as a blocking call probably
        makes the most sense.
        """
        self.finish_moving()
        assert axis in [0, 1, 2]
        if self.verbose: print('PI XYZ stage homing single axis:', axis)
        wait_block = C.POINTER(ACSC_WaitBlock)()
        label = 'home%d' % axis
        dll.run_buffer(
            self.handle,
            C.c_int32(1), # buffer 1 is the homing routine
            C.c_char_p(bytes(label, encoding='ascii')),
            wait_block)
        if blocking:
            if self.verbose: print('Finishing XYZ stage homing...')
            self.finish_program(2) # buffer 1 is the homing program
            if self.very_verbose: print('XYZ stage homing program finished.')
        return None

    def center_all_axes(self, blocking=True):
        """Moves all three axes to their center position."""
        self._check_errors_and_messages()
        if self.verbose: print('Centering all axes on XYZ stage.')
        x_center = (self.x_max - self.x_min) / 2 + self.x_min
        y_center = (self.y_max - self.y_min) / 2 + self.y_min
        z_center = (self.z_max - self.z_min) / 2 + self.z_min
        self.move_absolute(x=x_center, y=y_center, z=z_center,
                           blocking=blocking)
        return None

    def finish_moving(self):
        """Determines whether each axis is moving, and if it is, waits
        for that movement to finish before returning.
        
        This function uses a method wait_for_motion_end that won't
        return until the stage motion is completed. An alternative
        approach would be to repeatedly check the flag that indicates
        whether the stage is moving. However, this flag does not
        discriminate between major movements and the tiny adjustments
        necessary to maintain a position, so the "is moving" flag will
        sometimes be set for very short periods of time when the stage
        is mostly in position. It doesn't seem to cause problems to
        request a move during these adjustments, so I have opted to not
        check this flag after the wait_for_motion_end function returns.

        When a program is running, the "finish motion" function will
        return immediately (even if that axis is moving), so this
        function calls finish_program first to check that the homing
        routine is not running.
        """
        self.finish_program(2) # confirm homing routine has finished
        if self.very_verbose: print('Confirming XYZ motion is complete...')
        # check each axis one by one
        if self._get_is_motor_moving(axis=0): # X axis
            if self.very_verbose: print('Finishing X axis movement...')
            dll.wait_for_motion_end(self.handle,
                                    C.c_int32(0), # X axis
                                    C.c_int32(30000)) # timeout in ms
        if self._get_is_motor_moving(axis=1): # Y axis
            if self.very_verbose: print('Finishing Y axis movement...')
            dll.wait_for_motion_end(self.handle,
                                    C.c_int32(1), # Y axis
                                    C.c_int32(30000)) # timeout in ms
        if self._get_is_motor_moving(axis=2): # Z axis
            if self.very_verbose: print('Finishing Z axis movement...')
            dll.wait_for_motion_end(self.handle,
                                    C.c_int32(2), # Z axis
                                    C.c_int32(30000)) # timeout in ms
        if self.very_verbose: print('XYZ stage movement finished.')
        self._check_errors_and_messages()
        return None

    def finish_program(self, buffer=2):
        """Waits until the program loaded in a given buffer is finished.
        It defaults to checking program 2 (the homing routine), which is
        probably the only program we'll ever run.
        """
        if self._get_is_program_running(buffer):
            if self.very_verbose: print('Finishing XYZ stage program...')
            dll.wait_for_program_end(self.handle,
                                     C.c_int32(buffer),
                                     C.c_int32(30000)) # timeout in  ms
            assert self._get_is_program_running(buffer) != True
            if self.very_verbose: print('XYZ stage program is complete.')
        else:
            if self.very_verbose: print('XYZ stage program already complete.')
        return None

    def move_absolute(self, x=None, y=None, z=None, blocking=True):
        """Finishes any existing moves and then executes a move to the
        absolute coordinates specified by x, y, and z. If "None" is
        passed, it does not move that axis. The "blocking" flag dictates
        whether the function will wait until the stage move is completed
        before returning.
        """
        self.finish_moving()
        if self.very_verbose: print('Starting XYZ stage motion')
        if x is not None:
            if (x > self.x_max) or (x < self.x_min):
                raise ValueError('Attempted to move out of range in X.')
            self.x = float(x) # note that this won't be instantaneously correct
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.acsc_ToPoint(
                self.handle,
                C.c_int(0), # int specifying the flags - TODO: do I need any?
                C.c_int(0), # axis 0 = X
                C.c_double(x), # position to move to
                wait_block) # asynchronous
        if y is not None:
            if (y > self.y_max) or (y < self.y_min):
                raise ValueError('Attempted to move out of range in Y.')
            self.y = float(y) # note that this won't be instantaneously correct
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.acsc_ToPoint(
                self.handle,
                C.c_int(0), # int specifying the flags - TODO: do I need any?
                C.c_int(1), # axis 1 = Y
                C.c_double(y), # position to move to
                wait_block) # asynchronous
        if z is not None:
            if (z > self.z_max) or (z < self.z_min):
                raise ValueError('Attempted to move out of range in Z.')
            self.z = float(z) # note that this won't be instantaneously correct
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.acsc_ToPoint(
                self.handle,
                C.c_int(0), # int specifying the flags - TODO: do I need any?
                C.c_int(2), # axis 2 = Z
                C.c_double(z), # position to move to
                wait_block) # asynchronous
        if blocking:
            self.finish_moving()
        return None

    def move_relative(self, x=None, y=None, z=None, blocking=True):
        """Executes a relative move in the specified amount of mm from
        the stage's INSTANTANEOUS (i.e. current) position. Does NOT
        enforce that motion ends before the new position is calculated.
        Movement is only executed for axes to which a value other than
        "None" is passed. The "blocking" parameter indicates whether the
        function proceeds asynchronously or returns only when movement
        is complete.
        """
        self.get_position() # updates the internal variables w/truth
        # We don't sanitize inputs here or update internal position
        # beliefs -- move_absolute will do this.
        if x != None:
            new_x = self.x + float(x)
        else:
            new_x = None
        if y != None:
            new_y = self.y + float(y)            
        else:
            new_y = None
        if z != None:
            new_z = self.z + float(z)
        else:
            new_z = None
        if self.very_verbose: print('Initializing relative stage motion')
        self.move_absolute(new_x, new_y, new_z, blocking = blocking)
        return None

    def halt(self, axis):
        """Halts the current motion for the specified axis. This
        function can be used for "long range sending," where the axis is
        sent (SLOWLY) to the end of range and then halted when the user
        is at the desired location, e.g. when a key is released.
        """
        assert axis in [0, 1, 2]
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.halt(self.handle,
                 C.c_int32(axis),
                 wait) # asynchronous - goes immediately
        if self.very_verbose: print('Axis %d halted' % axis)
        self.get_position() # figure out where we ended up
        return
                 

    def get_position(self):
        """Returns the current position of the XYZ stage and updates the
        internal variables. If the stage is currently moving, it will
        return the instantaneous position, not the intended destination.
        """
        if self.very_verbose: print('PI XYZ stage getting position')
        x = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_position(self.handle, C.c_int32(0), x, wait) # X axis
        self.x = x.value
        y = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_position(self.handle, C.c_int32(1), y, wait) # Y axis
        self.y = y.value
        z = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_position(self.handle, C.c_int32(2), z, wait) # Z axis
        self.z = z.value
        if self.verbose:
            print('Stage position (mm): x', np.round(self.x, 6),
                  ', y', np.round(self.y, 6), ', z', np.round(self.z, 6))
        return self.x, self.y, self.z

    def get_velocity(self):
        """Returns the default velocities in mm/s for the X, Y, and Z
        axes, not the rate at which the axes are currently moving.
        """
        if self.very_verbose: print('PI XYZ stage getting velocity')
        vx = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_velocity(self.handle, C.c_int32(0), vx, wait) # X axis
        self.vx = vx.value
        vy = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_velocity(self.handle, C.c_int32(1), vy, wait) # Y axis
        self.vy = vy.value
        vz = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_velocity(self.handle, C.c_int32(2), vz, wait) # Z axis
        self.vz = vz.value
        if self.verbose:
            print('Stage velocity (mm/s): x', np.round(self.vx, 6),
                  ', y', np.round(self.vy, 6), ', z', np.round(self.vz, 6))
        return self.vx, self.vy, self.vz

    def get_acceleration(self):
        """Returns the default accelerations (mm/s^2) for the X, Y, and
        Z axes, not the instantaneous acceleration value. Although
        acceleration and deceleration are separate settings in the C
        API, we choose to combine them here for simplicity.
        """
        if self.very_verbose: print('PI XYZ stage getting acceleration')
        # Get values for the X axis
        ax = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_acceleration(self.handle, C.c_int32(0), ax, wait) # X accel
        _ax = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_deceleration(self.handle, C.c_int32(0), _ax, wait) # X decel
        assert round(ax.value) == round(_ax.value)
        self.ax = ax.value
        # Get values for the Y axis
        ay = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_acceleration(self.handle, C.c_int32(1), ay, wait) # Y accel
        _ay = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_deceleration(self.handle, C.c_int32(1), _ay, wait) # Y decel
        assert round(ay.value) == round(_ay.value)
        self.ay = ay.value
        # Get values for the Z axis
        az = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_acceleration(self.handle, C.c_int32(2), az, wait) # Z accel
        _az = C.c_double()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.get_deceleration(self.handle, C.c_int32(2), _az, wait) # Z decel
        assert round(az.value) == round(_az.value)
        self.az = az.value
        if self.verbose:
            print('Stage acceleration (mm/s^2): x', np.round(self.ax, 6),
                  ', y', np.round(self.ay, 6), ', z', np.round(self.az, 6))
        return self.ax, self.ay, self.az

    def set_velocity(self, vx=None, vy=None, vz=None):
        """Sets the movement velocity of the XYZ stage (mm/s).
        Velocities that are not passed are unchanged.
        
        If you call this while the stage is moving, it will not affect
        the existing motion but will apply on the next "move" call.
        """
        if vx is not None:
            if vx > self.vx_max or vx < self.vx_min:
                raise ValueError(
                    'Attempted to set out-of-range velocity for X axis.')
            self.vx = vx
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_velocity(self.handle,
                             C.c_int32(0), # X axis
                             C.c_double(vx),
                             wait_block) # asynchronous
            if self.verbose: print('New X velocity setting (mm/s):', vx)
        if vy is not None:
            if vy > self.vy_max or vy < self.vy_min:
                raise ValueError(
                    'Attempted to set out-of-range velocity for Y axis.')
            self.vy = vy
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_velocity(self.handle,
                             C.c_int32(1), # Y axis
                             C.c_double(vy),
                             wait_block) # asynchronous
            if self.verbose: print('New Y velocity setting (mm/s):', vy)
        if vz is not None:
            if vz > self.vz_max or vz < self.vz_min:
                raise ValueError(
                    'Attempted to set out-of-range velocity for Z axis.')
            self.vz = vz
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_velocity(self.handle,
                             C.c_int32(2), # Z axis
                             C.c_double(vz),
                             wait_block) # asynchronous
            if self.verbose: print('New Z velocity setting (mm/s):', vz)
        return None

    def set_acceleration(self, ax=None, ay=None, az=None):
        """Sets the acceleration and deceleration of the PI XYZ stage in
        mm/s^2. Axes whose accelerations are unspecified remain
        unchanged. Note that acceleration and deceleration are
        separately set-able parameters. Here, I'm opting to set them
        together, since I imagine we'll want symmetrical acceleration
        and deceleration.

        Mendel from PI recommends a settings of acceleration that is
        7.5-10x the velocity setting for smooth motion (& a jerk setting
        that is 7.5-10x the acceleration, although I do not enable
        setting jerk in the python API).

        Note that this acceleration value persists during the "home"
        routine, although the velocity does not. If you set the
        deceleration too low during the home routine, you will get a
        critical position error (it won't slow down enough to keep from
        going out of range). I'm keeping the acceleration at 2500 for
        now (not Mendel's recommendation for smooth motion) for this
        reason. You could also imagine a fix that sets a homing
        acceleration in the home buffer, but I haven't implemented that
        yet.
        
        If you call this while the stage is moving, it will not affect
        the existing motion but will apply on the next "move" call.
        """
        if ax is not None:
            if ax < self.ax_min or ax > self.ax_max:
                raise ValueError(
                    'Attempted to set out-of-range acceleration for X axis.')
            self.ax = ax
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_acceleration(self.handle,
                                 C.c_int32(0), # X axis
                                 C.c_double(ax),
                                 wait_block) # asynchronous
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_deceleration(self.handle,
                                 C.c_int32(0), # X axis
                                 C.c_double(ax),
                                 wait_block) # asynchronous            
            if self.verbose: print('New X acceleration setting (mm/s^2):', ax)
        if ay is not None:
            if ay < self.ay_min or ay > self.ay_max:
                raise ValueError(
                    'Attempted to set out-of-range acceleration for Y axis.')
            self.ay = ay
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_acceleration(self.handle,
                                 C.c_int32(1), # Y axis
                                 C.c_double(ay),
                                 wait_block) # asynchronous
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_deceleration(self.handle,
                                 C.c_int32(1), # Y axis
                                 C.c_double(ay),
                                 wait_block) # asynchronous
            if self.verbose: print('New Y acceleration setting (mm/s^2):', ay)
        if az is not None:
            if az < self.az_min or az > self.az_max:
                raise ValueError(
                    'Attempted to set out-of-range acceleration for Z axis.')
            self.az = az
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_acceleration(self.handle,
                                 C.c_int32(2), # Z axis
                                 C.c_double(az),
                                 wait_block) # asynchronous
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.set_deceleration(self.handle,
                                 C.c_int32(2), # Z axis
                                 C.c_double(az),
                                 wait_block) # asynchronous
            if self.verbose: print('New Z acceleration setting (mm/s^2):', az)
        return None

    def reset_to_default_velocity_acceleration(self):
        """Sets stage to factory default velocity and acceleration."""
        if self.verbose:
            print('Resetting to default velocity and acceleration.')
        self.set_velocity(25, 25, 20)
        self.set_acceleration(2500, 2500, 2500)
        return None
                                                      
    def close(self,  disable_motors=True):
        """Finishes movement, disables motors, and closes ethernet
        connection to the PI XYZ stage.
        """
        if self.verbose: print('\nBeginning PI XYZ stage disconnection.')
        if disable_motors:
            if self.verbose: print('Disabling PI XYZ motors')
            self._disable_motors() # this fxn also checks that we aren't moving
        else:
            print('Leaving motors in enabled state.')
            # TODO: check if motors are actually enabled?
        dll.close_message_buffer(self.handle)
        dll.close_ethernet_comm(self.handle)
        print('Closed PI XYZ Stage')
        return None


    def _get_motor_state(self, axis):
        """Returns the value of the motor state parameter, which is a
        series of bitwise flags indicating motor status. Note that
        calling this function during the homing routine results in an
        error.
        """
        state = C.c_int32()
        wait_block = C.POINTER(ACSC_WaitBlock)()
        dll.get_motor_state(self.handle,
                            C.c_int32(axis),
                            state,
                            wait_block) # asynchronous
        return state.value

    def _get_motor_flags(self, axis):
        assert axis in [0, 1, 2]
        var_name = C.c_char_p(b'MFLAGS')
        motor_flags = C.c_int32()
        wait = C.POINTER(ACSC_WaitBlock)()
        dll.read_integer(self.handle,
                         -1, # ACSC_NONE
                         var_name,
                         C.c_int32(axis), C.c_int32(axis+1),
                         -1, -1, # no second dimension to variable
                         motor_flags, # where the results will be stored
                         wait) # asynchronous
        return motor_flags.value
    
    def _get_is_motor_homed_and_enabled(self, axis):
        """Checks whether the specified axis is both homed and enabled
        (i.e. is ready to receive motion commands).
        """
        motor_flags = self._get_motor_flags(axis)
        # mflags bit 3 is the home status
        flags_shifted = motor_flags >> 3
        is_homed = ((flags_shifted & 1) == 1)
        if self.very_verbose: print('Axis %d homed?:' % axis, is_homed)
        # motor state least significant bit tests if enabled
        motor_state = self._get_motor_state(axis)
        is_enabled = ((motor_state & 1) == 1)
        if self.very_verbose: print('Axis %d enabled?:' % axis, is_enabled)
        if is_homed and is_enabled:
            return True
        else:
            return False

    def _get_is_motor_moving(self, axis):
        """Examines bits in the motor state flag that correspond to
        motor motion and the motor being "in position." It asserts that
        these bits should always be opposite; I have yet to find a case
        that contradicts that. It returns the motor_is_moving flag.
        """
        motor_state = self._get_motor_state(axis)
        # The 5th bit (0x00000020) is the motor is moving bit flag
        motor_shifted = motor_state >> 5
        is_moving = ((motor_shifted & 1) == 1)
        if self.very_verbose:
            print('Motor for axis %d is moving?' % axis, is_moving)
        # The 4th bit (0x00000010) is whether motor has settled. We
        # expect this will be the opposite of bit 5, but let's check it
        # to be sure.
        in_pos_shifted = motor_state >> 4
        is_in_pos = ((in_pos_shifted & 1) == 1)
        if self.very_verbose:
            print('Motor for axis %d in position?' % axis, is_in_pos)
        assert is_moving != is_in_pos # should always be opposites
        return is_moving

    def _get_is_program_running(self, buffer=1):
        """Retrieves and bit masks the program state flag to determine
        whether a program is currently running. Unlike many other
        functions, this seems to "behave" when the homing routine is
        running and can accurately report whether the stage is running a
        homing program.
        """
        prog_state = C.c_int32()
        wait_block = C.POINTER(ACSC_WaitBlock)()
        dll.get_program_state(
            self.handle,
            C.c_int32(buffer), # buffer 1 is the homing routine
            prog_state,
            wait_block) # asynchronous
        # the 0x00000002 bit of the program state flag is "is running"
        is_running = (((prog_state.value >> 1) & 1) == 1)
        # bit mask to get the program state
        if self.very_verbose:
            print('Program on buffer %d is running?' % buffer, is_running)
        return is_running
        
    def _disable_motors(self, axes=[0, 1, 2]):
        """Confirms movement and programs have stopped and then
        disables the motors for the corresponding axes. Used during
        shutdown routine.
        """
        self.finish_moving()
        for axis in axes:
            if axis not in [0, 1, 2]:
                raise ValueError('Attempted to disable nonexistent PI axis')
            wait_block = C.POINTER(ACSC_WaitBlock)()
            dll.disable_motor(self.handle,
                              axis, # the axis to disable
                              wait_block) # asynchronous
            if self.verbose: print('Disabled axis %i motor' % axis)
        return None

    def _check_errors_and_messages(self):
        """Checks for unsolicited messages from the stage in the message
        buffer. It also checks for errors from functions and motor
        errors (limits, overcurrents, safety stops, etc.) This allows
        for better handling of asynchronously generated errors.
        
        Because of how the ACSC_WaitBlock works, I'm not sure that this
        function can actually catch errors from asynchronous function
        calls. It's something to look more into in the future.
        """
        bytes_received = C.c_int32()
        buf = C.c_char_p(b'*'*2000)
        dll.get_message(
            self.handle,
            buf, # pointer to the message buffer
            1999, # size of the buffer
            bytes_received,
            C.c_int32(1)) # "BOOL" to clear buffer after reading
        if self.very_verbose and bytes_received == 0:
            print('\nNo unsolicited messages on XYZ stage.')
        if self.very_verbose and bytes_received != 0:
            print('\nFetching unsolicited messages from XYZ stage.')
            print(buf.value.decode('ascii')[:bytes_received.value])
        # Let's also check the function error logs
        error_code = dll.get_error_code()
        if error_code != 0: # if the last function returned an error
            error_string = C.c_char_p(b'*'*1000)
            ans = dll.get_error_string(
                self.handle,
                error_code,
                error_string,
                C.c_int32(999),
                bytes_received)
            if ans != 0:
                print('\nPI XYZ Stage error %i' % error_code)
                print(bytes_received.value, 'bytes parsed in error string:')
                print(error_string.value.decode('ascii'))
            else:
                print('PI XYZ Stage could not get error string.')
            raise RuntimeError(
                'PI XYZ error code: %i, see above for details' % error_code)
        # Finally, let's check for motor errors
        for axis in [0, 1, 2]:
            motor_error_code = C.c_int32()
            wait = C.POINTER(ACSC_WaitBlock)()
            dll.get_motor_error(self.handle,
                                C.c_int32(axis),
                                motor_error_code,
                                wait)
            if motor_error_code.value != 0:
                error_string = C.c_char_p(b'*'*1000)
                ans = dll.get_error_string(
                    self.handle,
                    motor_error_code,
                    error_string,
                    C.c_int32(999),
                    bytes_received)
                if ans != 0:
                    nb = bytes_received.value
                    print('\nPI XYZ motor error %i on axis %i'
                          % (motor_error_code.value, axis))
                    print(nb, 'bytes parsed in error string:')
                    print(error_string.value.decode('ascii')[:nb])
                else:
                    print('\nPI XYZ Stage could not get error string.')
                raise RuntimeError(
                    'PI XYZ motor error %i on axis %i, see above for details'
                    % (motor_error_code.value, axis))
            if self.very_verbose: print('No errors on XYZ axis %i' % axis)
        return None
                                    
## DLL Setup
## Set up error handling
dll.get_error_code = dll.acsc_GetLastError
dll.get_error_code.restype = C.c_int32

dll.get_error_string = dll.acsc_GetErrorString
dll.get_error_string.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_char_p,
    C.c_int32,
    C.POINTER(C.c_int32)]
dll.get_error_string.restype = C.c_int32

def check_error(result, func, arguments):
    """Checks the return values of dll calls. If they return 0
    (indicating error in this API), it calls a DLL get error function to
    get the most recent error that was sent. Note that, because many of
    the function calls are asynchronous, errors will be written to the
    ACSC_WaitBlock rather than to the function return value, and this
    check_error function may not catch them.
    """
    if func == dll.open_ethernet_comm:
        # This function doesn't follow pattern of the rest of the API
        if result == -1:
            print('Attempted ethernet connection to PI XYZ Stage')
            print('IP address:', arguments[0], ', Port:', arguments[1])
            raise OSError(
                'Failed to connect to PI XYZ Stage, see above details.')
    elif result == 0:
        # 0 means failure for all functions other than open port
        error_code = dll.get_error_code()
        error_string = C.c_char_p(b'*'*1000)
        bytes_received = C.c_int32()
        handle = arguments[0] # first arg to all fxns is device handle
        ans = dll.get_error_string(
            handle,
            error_code,
            error_string,
            C.c_int32(999),
            bytes_received)
        if ans != 0:
            print('\nPI XYZ Stage error %i' % error_code)
            print(bytes_received.value, 'bytes parsed in error string:')
            print(error_string.value.decode('ascii')[:bytes_received.value])
        else:
            print('PI XYZ Stage could not get error string.')
        raise RuntimeError(
            'PI XYZ error code: %i, see above for details' % error_code)
    return None

## Prototype other functions
# note that this open comms function is listed in the docs as returning
# an int, but the docs also say that it returns a HANDLE, which the
# ACSC.h defines as a Windows HANDLE (i.e. a C.c_void_p). So, I'm  using
# this function prototype where it returns a HANDLE, even though it's a
# bit against the convention of returning an int. It seems to work so
# far.
dll.open_ethernet_comm = dll.acsc_OpenCommEthernetTCP
dll.open_ethernet_comm.argtypes  = [
    C.c_char_p,
    C.c_int32]
dll.open_ethernet_comm.restype = C.c_void_p
dll.open_ethernet_comm.errcheck = check_error

dll.close_ethernet_comm = dll.acsc_CloseComm
dll.close_ethernet_comm.argtypes = [
    C.c_void_p]
dll.close_ethernet_comm.restype = C.c_void_p
dll.close_ethernet_comm.errcheck = check_error

dll.get_position = dll.acsc_GetFPosition
dll.get_position.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_double),
    C.POINTER(ACSC_WaitBlock)]
dll.get_position.restype = C.c_int32
dll.get_position.errcheck = check_error

dll.get_program_state = dll.acsc_GetProgramState
dll.get_program_state.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_int32),
    C.POINTER(ACSC_WaitBlock)]
dll.get_program_state.restype = C.c_int32

dll.get_motor_state = dll.acsc_GetMotorState
dll.get_motor_state.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_int32),
    C.POINTER(ACSC_WaitBlock)]
dll.get_motor_state.restype = C.c_int32
dll.get_motor_state.errcheck = check_error

dll.get_serial_number = dll.acsc_GetSerialNumber
dll.get_serial_number.argtypes = [
    C.c_void_p,
    C.c_char_p,
    C.c_int32,
    C.POINTER(C.c_int32),
    C.POINTER(ACSC_WaitBlock)]
dll.get_serial_number.restype = C.c_int32
dll.get_serial_number.errcheck = check_error

dll.get_velocity = dll.acsc_GetVelocity
dll.get_velocity.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_double),
    C.POINTER(ACSC_WaitBlock)]
dll.get_velocity.restype = C.c_int32
dll.get_velocity.errcheck = check_error

dll.get_acceleration = dll.acsc_GetAcceleration
dll.get_acceleration.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_double),
    C.POINTER(ACSC_WaitBlock)]
dll.get_acceleration.restype = C.c_int32
dll.get_acceleration.errcheck = check_error

dll.get_deceleration = dll.acsc_GetDeceleration
dll.get_deceleration.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_double),
    C.POINTER(ACSC_WaitBlock)]
dll.get_deceleration.restype = C.c_int32
dll.get_deceleration.errcheck = check_error    

dll.set_velocity = dll.acsc_SetVelocity
dll.set_velocity.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_double,
    C.POINTER(ACSC_WaitBlock)]
dll.set_velocity.restype = C.c_int32
dll.set_velocity.errcheck = check_error

dll.set_acceleration = dll.acsc_SetAcceleration
dll.set_acceleration.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_double,
    C.POINTER(ACSC_WaitBlock)]
dll.set_acceleration.restype = C.c_int32
dll.set_acceleration.errcheck = check_error

dll.set_deceleration = dll.acsc_SetDeceleration
dll.set_deceleration.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_double,
    C.POINTER(ACSC_WaitBlock)]
dll.set_deceleration.restype = C.c_int32
dll.set_deceleration.errcheck = check_error

dll.move_to_point = dll.acsc_ToPoint
dll.move_to_point.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_int32,
    C.c_double,
    C.POINTER(ACSC_WaitBlock)]
dll.move_to_point.restype = C.c_int32
dll.move_to_point.errcheck = check_error

dll.wait_for_motion_end = dll.acsc_WaitMotionEnd
dll.wait_for_motion_end.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_int32]
dll.wait_for_motion_end.restype = C.c_int32
dll.wait_for_motion_end.errcheck = check_error

dll.wait_for_program_end = dll.acsc_WaitProgramEnd
dll.wait_for_program_end.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_int32]
dll.wait_for_program_end.restype = C.c_int32
dll.wait_for_program_end.errcheck = check_error

dll.disable_motor = dll.acsc_Disable
dll.disable_motor.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(ACSC_WaitBlock)]
dll.disable_motor.restype = C.c_int32
dll.disable_motor.errcheck = check_error

dll.halt = dll.acsc_Halt
dll.halt.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(ACSC_WaitBlock)]
dll.halt.restype = C.c_int32
dll.halt.errcheck = check_error

dll.run_buffer = dll.acsc_RunBuffer
dll.run_buffer.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_char_p,
    C.POINTER(ACSC_WaitBlock)]
dll.run_buffer.restype = C.c_int32
dll.run_buffer.errcheck = check_error

dll.open_message_buffer = dll.acsc_OpenMessageBuffer
dll.open_message_buffer.argtypes = [
    C.c_void_p,
    C.c_int32]
# Note that I haven't been able to make this pointer work - function
# seems to return NULL even when it does successfully initialize a
# buffer.
dll.open_message_buffer.restype = C.POINTER(ACSC_HistoryBuffer)
dll.open_message_buffer.errcheck = check_error

dll.close_message_buffer = dll.acsc_CloseMessageBuffer
dll.close_message_buffer.argtypes = [C.c_void_p]
dll.close_message_buffer.restype = C.c_int32
dll.close_message_buffer.errcheck = check_error

dll.get_message = dll.acsc_GetMessage
dll.get_message.argtypes = [
    C.c_void_p,
    C.c_char_p,
    C.c_int32,
    C.POINTER(C.c_int32),
    C.c_int32] # in the windows data types API, BOOL is an int...
dll.get_message.restype = C.c_int32
dll.get_message.errcheck = check_error

dll.get_motor_error = dll.acsc_GetMotorError
dll.get_motor_error.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(C.c_int32),
    C.POINTER(ACSC_WaitBlock)]
dll.get_motor_error.restype = C.c_int32
dll.get_motor_error.errcheck = check_error

dll.read_integer = dll.acsc_ReadInteger
dll.read_integer.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.c_void_p,
    C.c_int32,
    C.c_int32,
    C.c_int32,
    C.c_int32,
    C.POINTER(C.c_int32),
    C.POINTER(ACSC_WaitBlock)]
dll.read_integer.restype = C.c_int32
dll.read_integer.errcheck = check_error

dll.clear_faults = dll.acsc_FaultClear
dll.clear_faults.argtypes = [
    C.c_void_p,
    C.c_int32,
    C.POINTER(ACSC_WaitBlock)]
dll.clear_faults.restype = C.c_int32
dll.clear_faults.errcheck = check_error

if __name__ == '__main__':
    ## Test basic functionality of the stage
    import time
    import random
    
    xyz_stage = PI_Custom_XYZ_Stage(verbose=True, very_verbose=False)

    xyz_stage.get_position()
    
##    xyz_stage.set_velocity(vx=20, vy=20, vz=10)
##    xyz_stage.get_velocity()
##    xyz_stage.set_acceleration(ax=2000, ay=2000, az=1000)
##    xyz_stage.get_acceleration()
##
##    num_moves = 5
##    for mov in range(num_moves):
##        x_new = random.uniform(xyz_stage.x_min, xyz_stage.x_max)
##        y_new = random.uniform(xyz_stage.y_min, xyz_stage.y_max)
##        z_new = random.uniform(xyz_stage.z_min, xyz_stage.z_max)
##        xyz_stage.move_absolute(x=x_new, y=y_new, z=z_new, blocking=False)
##
##    xyz_stage.move_absolute(x=0, y=0, z=5, blocking=False)
##    time.sleep(1)
##    xyz_stage.halt(axis=0)
  
    # let's go back to some reasonable settings
    xyz_stage.reset_to_default_velocity_acceleration()
    xyz_stage.center_all_axes(blocking=True)
    

    xyz_stage.close()

