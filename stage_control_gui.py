import os
from datetime import datetime
import tkinter as tk

import pi_custom_xyz as xyz

class GuiStageControl:
    def __init__(self, root, verbose=False):
        self.root = root
        self.verbose = verbose
        self.stage = xyz.PI_Custom_XYZ_Stage(verbose=verbose)
        self.stage.set_velocity(vx=5, vy=5, vz=10)
        self.z_step_size = 2e-4
        self.xy_step_size = 1e-3 

        ## Create a label frame for XY travel control
        self.travel_frame_xy = tk.LabelFrame(self.root, text="XY Control",
                                             width=600, height=100)
        self.travel_frame_xy.grid_propagate(0)
        self.travel_frame_xy.grid(row=2, column=0, rowspan=1, columnspan=3,
                                  padx=20, pady=10, sticky='nw')
        # XY slider
        self.xy_label = tk.Label(self.travel_frame_xy, text="XY speed")
        self.xy_label.grid(row=0, column=0, sticky='se', padx=5)
        self.xy_speed = tk.IntVar(self.travel_frame_xy, 10)
        self.xy_speed_scale = tk.Scale(self.travel_frame_xy,
                                       variable = self.xy_speed,
                                       orient='horizontal',
                                       length=300)
        self.xy_speed_scale.bind("<ButtonRelease-1>", self.update_xy_speed)
        self.xy_speed_scale.grid(row=0, column=1, rowspan=1, columnspan=2)
        # Motion mode controls for XY
        self.travel_mode_xy = tk.StringVar(self.travel_frame_xy,
                                           value="inching")
        inching_button_xy = tk.Radiobutton(self.travel_frame_xy,
                                           text="Inching Mode (Slow)",
                                           variable=self.travel_mode_xy,
                                           value="inching",
                                           command=self.update_travel_mode_xy)
        inching_button_xy.grid(row=0, column=4, sticky='nw', padx=20)
        cont_button_xy = tk.Radiobutton(self.travel_frame_xy,
                                        text="Continuous Mode (Fast)",
                                        variable=self.travel_mode_xy,
                                        value="continuous",
                                        command=self.update_travel_mode_xy)
        cont_button_xy.grid(row=1, column=4, sticky='nw', padx=20)
        ## Create a label frame for Z travel control
        self.travel_frame_z = tk.LabelFrame(self.root, text="Z Control",
                                            width=600, height=100)
        self.travel_frame_z.grid_propagate(0)
        self.travel_frame_z.grid(row=3, column=0, rowspan=1, columnspan=3,
                                 padx=20, pady=10, sticky='nw')
        # Z slider
        self.z_label = tk.Label(self.travel_frame_z, text="Z speed")
        self.z_label.grid(row=0, column=0, sticky='se', padx=5)
        self.z_speed = tk.IntVar(self.travel_frame_z, 10)
        self.z_speed_scale = tk.Scale(self.travel_frame_z,
                                      variable = self.z_speed,
                                      orient='horizontal',
                                      length=300)
        self.z_speed_scale.bind("<ButtonRelease-1>", self.update_z_speed)
        self.z_speed_scale.grid(row=0, column=1, rowspan=1, columnspan=2)
        # Motion mode controls for Z
        self.travel_mode_z = tk.StringVar(self.travel_frame_z,
                                           value="inching")
        inching_button_z = tk.Radiobutton(self.travel_frame_z,
                                           text="Inching Mode (Slow)",
                                           variable=self.travel_mode_z,
                                           value="inching",
                                           command=self.update_travel_mode_z)
        inching_button_z.grid(row=0, column=4, sticky='nw', padx=20)
        cont_button_z = tk.Radiobutton(self.travel_frame_z,
                                        text="Continuous Mode (Fast)",
                                        variable=self.travel_mode_z,
                                        value="continuous",
                                        command=self.update_travel_mode_z)
        cont_button_z.grid(row=1, column=4, sticky='nw', padx=20)

        ## Label frame that contains the best estimates of position
        self.position_frame = tk.LabelFrame(self.root,
                                            text="Approx. Current Position")
        self.position_frame.grid(row=2, column=4, rowspan=2, columnspan=1,
                                 padx=20, pady=10, sticky='nw')
        self.x_est = tk.StringVar(
            self.position_frame,
            value="X: " + str(round(self.stage.x, 3)) + " mm")
        self.x_pos_label = tk.Label(self.position_frame,
                                    textvariable=self.x_est)
        self.x_pos_label.grid(row=0, column=0, padx=5, pady=10, sticky='w')
        self.y_est = tk.StringVar(
            self.position_frame,
            value="Y: " + str(round(self.stage.y, 3)) + " mm")
        self.y_pos_label = tk.Label(self.position_frame,
                                    textvariable=self.y_est)
        self.y_pos_label.grid(row=1, column=0, padx=5, pady=10, sticky='w')
        self.z_est = tk.StringVar(
            self.position_frame,
            value="Z: " + str(round(self.stage.z, 5)) + " mm")
        self.z_pos_label = tk.Label(self.position_frame,
                                    textvariable=self.z_est)
        self.z_pos_label.grid(row=2, column=0, padx=5, pady=10, sticky='w')

        ## Label frame that contains the movement buttons
        w = 15
        h = 5
        pad = 5
        self.arrow_frame = tk.LabelFrame(self.root,
                                         text='Stage Controller Buttons',
                                         width=600, height=325)
        self.arrow_frame.grid_propagate(0)
        self.arrow_frame.grid(row=0, column=0, padx=20, pady=10, sticky='nw',
                              rowspan=2, columnspan=3)
        # FORWARD
        fwd = tk.Button(self.arrow_frame, text="FWD (∧)",
                        height=h, width=w)
        fwd.bind('<ButtonPress-1>',
                 func=lambda event: self.move(button='fwd'))
        fwd.bind('<ButtonRelease-1>',
                 func=lambda event: self.stop_moving(axis=1))
        fwd.grid(row=0, column=1, padx=pad, pady=pad, sticky='n')
        # BACK
        back = tk.Button(self.arrow_frame, text="BACK (∨)",
                         height=h, width=w)
        back.bind('<ButtonPress-1>',
                  func=lambda event: self.move(button='back'))
        back.bind('<ButtonRelease-1>',
                  func=lambda event: self.stop_moving(axis=1))
        back.grid(row=2, column=1, padx=pad, pady=pad, sticky='n')
        # RIGHT
        right = tk.Button(self.arrow_frame, text="RIGHT (>)",
                          height=h, width=w)
        right.bind('<ButtonPress-1>',
                   func=lambda event: self.move(button='r'))
        right.bind('<ButtonRelease-1>',
                   func=lambda event: self.stop_moving(axis=0))
        right.grid(row=1, column=2, padx=pad, pady=pad, sticky='n')
        # LEFT
        left = tk.Button(self.arrow_frame, text="LEFT (<)",
                         height=h, width=w)
        left.bind('<ButtonPress-1>',
                  func=lambda event: self.move(button='l'))
        left.bind('<ButtonRelease-1>',
                  func=lambda event: self.stop_moving(axis=0))
        left.grid(row=1, column=0, padx=pad, pady=pad, sticky='n')
        # UP
        up = tk.Button(self.arrow_frame, text = "UP (PgUp)",
                       height=h, width=w)
        up.bind('<ButtonPress-1>',
                func=lambda event: self.move(button='up'))
        up.bind('<ButtonRelease-1>',
                func=lambda event: self.stop_moving(axis=2))
        up.grid(row=0, column=3, padx=pad + 20, pady=pad, sticky='n')
        # DOWN
        down = tk.Button(self.arrow_frame, text = "DOWN (PgDown)",
                         height=h, width=w)
        down.bind('<ButtonPress-1>',
                  func=lambda event: self.move(button='down'))
        down.bind('<ButtonRelease-1>',
                  func=lambda event: self.stop_moving(axis=2))
        down.grid(row=1, column=3, padx=pad + 20, pady=pad, sticky='n')
        # checkbox within this label frame to enable the keys
        self.keys_enabled = tk.BooleanVar(self.root, 0)
        enable = tk.Checkbutton(self.arrow_frame,
                                text='Enable keyboard control',
                                onvalue=1, offvalue=0,
                                variable=self.keys_enabled,
                                command=self.update_keyboard_control)
        enable.grid(row=0, column=0, padx=pad, pady=pad, rowspan=1,
                    columnspan=1)
        return None

    def update_keyboard_control(self):
        if self.keys_enabled.get():
            self.bind_movement_keys()
            print("Keyboard control of stage enabled")
        else:
            self.unbind_movement_keys()
            print("Keyboard control of stage disabled")
        return None

    def update_z_speed(self, event_params):
        print('new Z speed', self.z_speed.get())
        if self.travel_mode_z.get() == 'Scontinuous':
            # we update the actual stage velocity not step size
            velocity_mm_s = self.z_speed.get() / 10
            self.stage.set_velocity(vz=velocity_mm_s)
        return None

    def update_xy_speed(self, event_params):
        print('new XY speed', self.xy_speed.get())
        if self.travel_mode_xy.get() == 'continuous':
            # we update the actual stage velocity not step size
            velocity_mm_s = self.xy_speed.get() / 10
            self.stage.set_velocity(vx=velocity_mm_s, vy=velocity_mm_s)
        return None

    def update_travel_mode_xy(self):
        print("Changed XY travel mode to", self.travel_mode_xy.get())
        if self.travel_mode_xy.get() == 'inching':
            self.stage.set_velocity(vx=5, vy=5) # const. velocity, variable step
        elif self.travel_mode_xy.get() == 'continuous':
            velocity_mm_s = self.xy_speed.get() / 10 # variable velocity
            self.stage.set_velocity(vx=velocity_mm_s, vy=velocity_mm_s)
        return None

    def update_travel_mode_z(self):
        print("Changed Z travel mode to", self.travel_mode_z.get())
        if self.travel_mode_z.get() == 'inching':
            self.stage.set_velocity(vz=5) # const. velocity, variable step
        elif self.travel_mode_xy.get() == 'continuous':
            velocity_mm_s = self.z_speed.get() / 10 # variable velocity
            self.stage.set_velocity(vz=velocity_mm_s)
        return None

    def update_position_display(self):
        self.x_est.set("X: " + str(round(self.stage.x, 3)) + " mm")
        self.y_est.set("Y: " + str(round(self.stage.y, 3)) + " mm")
        self.z_est.set("Z: " + str(round(self.stage.z, 5)) + " mm")
        return None

    def move(self, event=None, button=None):
        if button is not None: # call came from GUI
            axis_dict = {'fwd': 'y', 'back': 'y', 'r': 'x', 'l': 'x',
                         'up': 'z', 'down': 'z'}
            direction_dict = {'fwd': 1, 'back': -1, 'r': 1, 'l': -1,
                              'up': 1, 'down': -1}
        if event is not None: # call came from keyboard
            axis_dict = {'Up': 'y', 'Down': 'y', 'Right': 'x', 'Left': 'x',
                         'Prior': 'z', 'Next': 'z'}
            direction_dict = {'Up': 1, 'Down': -1, 'Right': 1, 'Left': -1,
                              'Prior': 1, 'Next': -1}
            button = event.keysym
        xy_move = 0; z_move=0
        if axis_dict[button] == 'x' or axis_dict[button] == 'y':
            xy_move = 1
        elif axis_dict[button] == 'z':
            z_move = 1
        # Now, let's make the appropriate motion call
        if (xy_move and self.travel_mode_xy.get() == 'inching') or (
            z_move and self.travel_mode_z.get() == 'inching'):
            if xy_move:
                step_abs = self.xy_step_size * self.xy_speed.get()
            elif z_move:
                step_abs = self.z_step_size * self.z_speed.get()
            step = step_abs * direction_dict[button]
            move_dict = {axis_dict[button]: step}
            self.stage.move_relative(**move_dict, blocking=False)
            if self.verbose: print('Inching move:', axis_dict[button], step)
        elif (xy_move and self.travel_mode_xy.get() == 'continuous') or (
            z_move and self.travel_mode_z.get() == 'continuous'):
            letter_to_num = {'x': 0, 'y': 1, 'z': 2}
            axis_letter = axis_dict[button]
            axis_number = letter_to_num[axis_letter]
            if self.stage._get_is_motor_moving(axis_number):
                # we are already on a continuous move, don't repeat it
                return None
            if direction_dict[button] == -1: # go to neg limit
                destination_dict = {'x': self.stage.x_min,
                                    'y': self.stage.y_min,
                                    'z': self.stage.z_min}
            elif direction_dict[button] == 1: # go to pos limit
                destination_dict = {'x': self.stage.x_max,
                                    'y': self.stage.y_max,
                                    'z': self.stage.z_max}
            destination = destination_dict[axis_letter]
            current_pos = self.stage.get_position()
            if round(destination, 4) == round(current_pos[axis_number], 4):
                # we already got there, don't call move again
                print('Axis %s already at limit' % axis_letter)
                return None
            move_dict = {axis_letter: destination}
            if self.verbose: print('Sending', axis_letter, 'to', destination)
            self.stage.move_absolute(**move_dict, blocking=False)
        self.update_position_display()
        return None

    def stop_moving(self, event=None, axis=None):
        # TODO: does this get weird with inching motion?
        if event is not None: # call from the keyboard
            axis_dict = {'Up': 1, 'Down': 1, 'Right': 0, 'Left': 0,
                         'Prior': 2, 'Next': 2}
            axis = axis_dict[event.keysym]
        assert axis in [0, 1, 2]
        self.stage.halt(axis)
        if self.verbose: print('Axis %d halted.' % axis)
        self.update_position_display()
        return None
        
    def get_tkfocus(self, event):   # event is not used here (.bind)
        self.frame.focus_set()      # take from other widgets to force update
        return None

    def bind_movement_keys(self):
        # movement key press bindings
        self.root.bind(sequence="<KeyPress-Left>", func=self.move)
        self.root.bind(sequence="<KeyPress-Right>", func=self.move)
        self.root.bind(sequence="<KeyPress-Up>", func=self.move)
        self.root.bind(sequence="<KeyPress-Down>", func=self.move)
        self.root.bind(sequence="<KeyPress-Prior>", func=self.move)
        self.root.bind(sequence="<KeyPress-Next>", func=self.move)

        # key release bindings for sensitivity to duration of press
        self.root.bind(sequence="<KeyRelease-Left>", func=self.stop_moving)
        self.root.bind(sequence="<KeyRelease-Right>", func=self.stop_moving)
        self.root.bind(sequence="<KeyRelease-Up>", func=self.stop_moving)
        self.root.bind(sequence="<KeyRelease-Down>", func=self.stop_moving)
        self.root.bind(sequence="<KeyRelease-Prior>", func=self.stop_moving)
        self.root.bind(sequence="<KeyRelease-Next>", func=self.stop_moving)

        # ??? set up the booleans we will use in long travel mode to tell if
        # the key press is new or following a keypress of the same type
        return None

    def unbind_movement_keys(self):
        self.root.unbind("<KeyPress-Left>")
        self.root.unbind("<KeyPress-Right>")
        self.root.unbind("<KeyPress-Up>")       
        self.root.unbind("<KeyPress-Down>")
        self.root.unbind("<KeyPress-Prior>")
        self.root.unbind("<KeyPress-Next>")
        self.root.unbind("<KeyRelease-Left>")
        self.root.unbind("<KeyRelease-Right>")
        self.root.unbind("<KeyRelease-Up>")       
        self.root.unbind("<KeyRelease-Down>")
        self.root.unbind("<KeyRelease-Prior>")
        self.root.unbind("<KeyRelease-Next>")
        return None

    def close(self):
        print('CLOSING')
        self.stage.reset_to_default_velocity_acceleration()
        self.stage.close(disable_motors=False)
        # TODO: reset velocity so it's sane for the next homing routine?
        return None

if __name__ == '__main__':
    root = tk.Tk()
    root.title('Stage Control')

    gui_stage_control = GuiStageControl(root, verbose=False)


    quit_ = tk.Button(root, text="QUIT", command=root.destroy,
                      height=5, width=20)
    quit_.grid(row=1, column=4, padx=15, pady=5, sticky='n')

    root.mainloop()
    gui_stage_control.close()
    ##root.destroy()  ## TODO: do I need this?
