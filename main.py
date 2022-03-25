import PySimpleGUI as sg
import re
import random
from multiprocessing import shared_memory, Process
from multiprocessing.managers import BaseManager
import time
import sys

version = "0.0.88 Alfa (not registered)"
relase_date = "2021/04/08"
inputs_tuple = ("Temp 1", "Temp 2", "Barometer", "Tahometer", "Pump")
outputs_tuple = ("heater1", "heater2", "Motor1", "Motor2")
controllers_tuple = ("P", "PI", "PD", "PID")
controllers_model = []
for o in outputs_tuple:
    for c in controllers_tuple:
        controllers_model.append("./Graph/" + o + "_" + c + "_s.png")
controller_io = ("CONTW", "CONTS", "CONTZ", "CONTX", "CONTe", "CONTy", "CONTx")
parameters_tuple = ('-1PMIN-', '-1PMAX-', '-1NMIN-', '-1NMAX-', '-2PMIN-', '-2PMAX-', '-2NMIN-', '-2NMAX-', '-3PMIN-',
                    '-3PMAX-', '-3NMIN-', '-3NMAX-', '-4PMAX-', '-4PMIN-', '-4NMAX-', '-4NMIN-', '-5PPAR-', '-5NMIN-',
                    '-5NMAX-',
                    '-6PPAR-', '-7PPAR-', '-8PPAR-', '-9PPAR-', '-10PPAR-', '-11PPAR-', '-12PPAR-', '-13PPAR-',
                    '-13NMIN-', '-13NMAX-')  # [P] - parameter, [N] - noise
parameters_values = {}
for t in parameters_tuple:
    if re.search("-[0-9]+(N)", t):
        parameters_values[t] = "Disabled"
    else:
        parameters_values[t] = 0
controllers_values = {}
for o in outputs_tuple:
    for i in controller_io:
        controllers_values[o + "_" + i] = 0
formats_tuple = ('^[0-9]{1,3}\.[0-9]$', '^[0-9]{3,4}$', '^[0-9]{2}$', '^[0-9]{1,3}$', '^[0-9]{1,4}$',
                 '^[0-9]{1,2}:[0-9]{2}:[0-9]{2}$', '^[0-9]{1,2}$')
realtime_parameters_tuple = ('-1RT-', '-2RT-', '-3RT-', '-4RT-', '-5RT-', '-6RT-', '-7RT-')
realtime_parameters_values = {}
for r in realtime_parameters_tuple:
    realtime_parameters_values = 0
schematic_file = "./Graph/schematic768790.png"
schematic_small_file = "./Graph/schematic240247.png"
noise_disabled = True
in_last_con = '-Barometer-'  # (Barometer) CHANGE TO (Temp1) AFTER STABLE RELASE
schematic_opened = False
sim_paused = True
sim_rewind = False
hh = mm = ss = 0
actual_controller = controllers_tuple[0]
actual_object = outputs_tuple[0]
update_values = False


class Countdown_timer:
    def __init__(self):
        self.enabled = False
        self.hours = 0
        self.minutes = 0
        self.seconds = 0
        self.update = False

    def Update(self, timer):
        t = re.search("([0-9]{1,2}):([0-9]{2}):([0-9]{2})", timer)
        self.hours = int(t.group(1))
        self.minutes = int(t.group(2))
        self.seconds = int(t.group(3))
        self.update = True

    def Enable(self):
        self.enabled = True

    def Disable(self):
        self.enabled = False

    def Is_enabled(self):
        return self.enabled

    def Get_update_flag(self):
        return self.update

    def Get_time(self):
        self.update = False
        return self.hours, self.minutes, self.seconds


class DSP_container:
    def __init__(self):
        self.enable = False
        self.update_time = False
        self.update_data = False
        self.hours = 0
        self.minutes = 0
        self.seconds = 0
        self.sampling_rate = 0
        self.sampling_resolution = 0
        self.real_time = False
        self.Kp, self.Ki, self.Kd = 1.0, 0.0, 0.0
        self._P = self._I = self._D = 0
        self.setpoint = 0
        self.output = None
        self.last_output = None
        self.last_input = None
        self.dsp_pid_ready = False

    def Update_time(self, time):
        t = re.search("([0-9]{1,2}):([0-9]{2}):([0-9]{2})", time)
        self.hours = int(t.group(1))
        self.minutes = int(t.group(2))
        self.seconds = int(t.group(3))
        self.update_time = True

    def Update_data(self, sample_rate, sample_resolution, realtime=False):
        self.sampling_rate = sample_rate
        self.sampling_resolution = sample_resolution
        self.real_time = realtime
        self.update_data = True

    def Get_time(self):
        self.update_time = False
        return self.hours, self.minutes, self.seconds

    def Get_data(self):
        self.update_data = False

    def Get_time_update_flag(self):
        return self.update_time

    def Get_data_update_flag(self):
        return self.update_data

    def Enable(self):
        self.enable = True

    def Disable(self):
        self.enable = False

    def Is_enabled(self):
        return self.enable

    def DSP_Reset(self):
        pass

    def DSP_PID_Config(self, Kp=1.0, Ki=0.0, Kd=0.0, setpoint=0):
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self._P = self._I = self._D = 0
        self.setpoint = setpoint
        self.output = None
        self.last_output = None
        self.last_input = None
        self.dsp_pid_ready = False

    def DSP_PID_Compute(self, _input, dt=1):  # dt - timestamp in ms
        if dt <= 0:
            dt = 1
        error = self.setpoint - _input
        d_input = _input - (self.last_input if (self.last_input is not None) else _input)
        self._P = self.Kp * error
        self._I += self.Ki * error * dt
        self._D = -self.Kd * d_input / dt
        self.output = self._P + self._I + self._D
        self._last_output = self.output
        self._last_input = _input
        self.dsp_pid_ready = True

    def Is_PID_Ready(self):
        return self.dsp_pid_ready

    def Get_PID_Output(self):
        self.dsp_pid_ready = False
        return self.output


def main(cd_timer, DSP):
    global hh
    global mm
    global ss
    global sim_paused
    global sim_rewind
    global schematic_opened
    global in_last_con
    global noise_disabled
    global actual_controller
    global actual_object
    global update_values

    sg.theme('DarkGrey8')

    def inputs_tab():
        windows_buttons = [
            [
                sg.B("Inputs [I]", key="-INPUTS-", enable_events=True, disabled=True, size=(17, 1),
                     button_color='#AAAAAA'),
                sg.B("Controller [C]", key="-CONTROLLER-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Output [O]", key="-OUTPUT-", enable_events=True, size=(17, 1), button_color='#AAAAAA',
                     disabled=True),  # (disabled=True) REMOVE AFTER STABLE RELASE
                sg.B("Charts [H]", key="-CHARTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Info [F1]", key="-INFO-", enable_events=True, size=(17, 1), button_color='#AAAAAA')
            ]
        ]
        frame_1 = sg.Column([
            [sg.Frame("Select object",
                      [
                          [sg.Listbox(inputs_tuple, select_mode="LISTBOX_SELECT_MODE_SINGLE",
                                      size=(26, 5), no_scrollbar=True, enable_events=True, key='-SELINPUT-',
                                      disabled=True,  # (disabled=True) REMOVE AFTER STABLE RELASE
                                      tooltip="Select an object to enter its controller parameters",
                                      default_values=[inputs_tuple[2]])],
                          # (inputs_tuple[2]) CHANGE TO (inputs_tuple[0]) AFTER STABLE RELASE
                          [sg.Checkbox("Enable noise", key="-ENOISE-", enable_events=True,
                                       tooltip="Enable paramaters noise\nfor more reliable simulation",
                                       auto_size_text=True,
                                       checkbox_color=sg.DEFAULT_BACKGROUND_COLOR)]
                      ]
                      )], [
                sg.Frame("Simulation data", [
                    [sg.Text("Uranium hexafluoride", auto_size_text=True),
                     sg.Input(tooltip="100 - 9999 [kg]", size=(7, 1), key=parameters_tuple[19],
                              disabled_readonly_background_color="#AAAAAA")],
                    [sg.Text("Tank1 capacity", auto_size_text=True),
                     sg.Input(tooltip="100 - 9999 [kg]", size=(7, 1), key=parameters_tuple[23],
                              disabled_readonly_background_color="#AAAAAA")],
                    [sg.Text("Tank2 capacity", auto_size_text=True),
                     sg.Input(tooltip="100 - 9999 [kg]", size=(7, 1), key=parameters_tuple[24], disabled=True,
                              disabled_readonly_background_color="#AAAAAA")],
                    # (disabled=True) REMOVE AFTER STABLE RELASE
                    [sg.Text("Tank3 capacity", auto_size_text=True),
                     sg.Input(tooltip="100 - 9999 [kg]", size=(7, 1), key=parameters_tuple[25], disabled=True,
                              disabled_readonly_background_color="#AAAAAA")],
                    # (disabled=True) REMOVE AFTER STABLE RELASE
                    [sg.Text("Ambient temperature", auto_size_text=True),
                     sg.Input(tooltip="0 - 100 [°C]", size=(7, 1), key=parameters_tuple[26],
                              disabled_readonly_background_color="#AAAAAA")],
                    [sg.Text("Min noise", size=(10, 1)), sg.Input(key=parameters_tuple[27], size=(5, 1), disabled=True,
                                                                  disabled_readonly_background_color="#AAAAAA",
                                                                  tooltip="0 - 100 [‰]")],
                    [sg.Text("Max noise", size=(10, 1)), sg.Input(key=parameters_tuple[28], size=(5, 1), disabled=True,
                                                                  disabled_readonly_background_color="#AAAAAA",
                                                                  tooltip="0 - 100 [‰]")]
                ])
            ]
        ], vertical_alignment='top')
        temp1 = [
            [sg.Frame("Temp1", [[
                sg.Text("Min Temperature", size=(14, 1)),
                sg.Input(tooltip="20.0 - 150.0 [°C]", key=parameters_tuple[0], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Max Temperature", size=(14, 1)),
                sg.Input(tooltip="20.0 - 150.0 [°C]", key=parameters_tuple[1], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Min noise", size=(10, 1)), sg.Input(key=parameters_tuple[2], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ], [
                sg.Text("Max noise", size=(10, 1)), sg.Input(key=parameters_tuple[3], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ]])]
        ]
        temp2 = [
            [sg.Frame("Temp2", [[
                sg.Text("Min Temperature", size=(14, 1)),
                sg.Input(tooltip="20.0 - 150.0 [°C]", key=parameters_tuple[4], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Max Temperature", size=(14, 1)),
                sg.Input(tooltip="20.0 - 150.0 [°C]", key=parameters_tuple[5], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Min noise", size=(10, 1)), sg.Input(key=parameters_tuple[6], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ], [
                sg.Text("Max noise", size=(10, 1)), sg.Input(key=parameters_tuple[7], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ]])]
        ]
        barmeter = [
            [sg.Frame("Barometer", [[
                sg.Text("Min Pressure", size=(14, 1)),
                sg.Input(tooltip="800 - 1500 [mBar]", key=parameters_tuple[8], size=(8, 1),
                         disabled_readonly_background_color="#AAAAAA")
            ], [
                sg.Text("Max Pressure", size=(14, 1)),
                sg.Input(tooltip="800 - 1500 [mBar]", key=parameters_tuple[9], size=(8, 1),
                         disabled_readonly_background_color="#AAAAAA")
            ], [
                sg.Text("Min noise", size=(10, 1)), sg.Input(key=parameters_tuple[10], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ], [
                sg.Text("Max noise", size=(10, 1)), sg.Input(key=parameters_tuple[11], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ]])]
        ]
        tahmeter = [
            [sg.Frame("Tahometer", [[
                sg.Text("Min RPM", size=(14, 1)),
                sg.Input(tooltip="10 - 80 [kRPM]", key=parameters_tuple[12], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Max RPM", size=(14, 1)),
                sg.Input(tooltip="10 - 80 [kRPM]", key=parameters_tuple[13], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Min noise", size=(10, 1)), sg.Input(key=parameters_tuple[14], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ], [
                sg.Text("Max noise", size=(10, 1)), sg.Input(key=parameters_tuple[15], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ]])]
        ]
        pump = [
            [sg.Frame("Pump", [[
                sg.Text("Efficiency", size=(14, 1)),
                sg.Input(tooltip="0 - 100 [%]", key=parameters_tuple[16], size=(8, 1), disabled=True,
                         disabled_readonly_background_color="#AAAAAA")
                # (disabled=True) REMOVE AFTER STABLE RELASE
            ], [
                sg.Text("Min noise", size=(10, 1)), sg.Input(key=parameters_tuple[17], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ], [
                sg.Text("Max noise", size=(10, 1)), sg.Input(key=parameters_tuple[18], size=(5, 1), disabled=True,
                                                             disabled_readonly_background_color="#AAAAAA",
                                                             tooltip="0 - 100 [‰]")
            ]])]
        ]
        frame_3 = sg.Column([
            [sg.Frame("Real-time adjustment", [
                [sg.Text("Temp1", size=(8, 1)),
                 sg.Slider(tooltip="%", range=(-50, 50), orientation='h', size=(14, 10),
                           key=realtime_parameters_tuple[0], default_value=0)],
                [sg.Text("Temp2", size=(8, 1)),
                 sg.Slider(tooltip="%", range=(-50, 50), orientation='h', size=(14, 10),
                           key=realtime_parameters_tuple[1], default_value=0)],
                [sg.Text("Barometer", size=(8, 1)),
                 sg.Slider(tooltip="%", range=(-50, 50), orientation='h', size=(14, 10),
                           key=realtime_parameters_tuple[2], default_value=0)],
                [sg.Text("Tahometer", size=(8, 1)),
                 sg.Slider(tooltip="%", range=(-50, 50), orientation='h', size=(14, 10),
                           key=realtime_parameters_tuple[3], default_value=0)],
                [sg.Text("Pump", size=(8, 1)),
                 sg.Slider(tooltip="%", range=(-50, 50), orientation='h', size=(14, 10),
                           key=realtime_parameters_tuple[4], default_value=0)],
                [sg.Checkbox("Pump", key=realtime_parameters_tuple[5], enable_events=True,
                             tooltip="Turn on/off main pump", auto_size_text=(1, 1),
                             checkbox_color=sg.DEFAULT_BACKGROUND_COLOR),
                 sg.Checkbox("Valve", key=realtime_parameters_tuple[6], enable_events=True,
                             tooltip="Turn on/off main valve", auto_size_text=(1, 1),
                             checkbox_color=sg.DEFAULT_BACKGROUND_COLOR)]
            ], key='-RTADJ-', visible=False, pad=((0, 0), (0, 0)))]
        ])
        frame_view_object = sg.Column([
            [sg.Frame("Simulation object", [[
                sg.Image(filename=schematic_small_file, key="-SIMIMG-", enable_events=True,
                         tooltip="Click to wiew in full size")
            ]])],
            [sg.Frame("Simulation parameters", [
                [sg.Text("Simulation time", size=(14, 1)),
                 sg.Input(tooltip="hh:mm:ss", size=(8, 1), key=parameters_tuple[20],
                          disabled_readonly_background_color="#AAAAAA")],
                [sg.Text("Sampling rate", size=(14, 1)),
                 sg.Input(tooltip="1 - 100 [sample per second]", size=(8, 1), key=parameters_tuple[21],
                          disabled_readonly_background_color="#AAAAAA")],
                [sg.Text("Sampling resolution", size=(14, 1)),
                 sg.Input(tooltip="8 - 18 [bits]", size=(8, 1), key=parameters_tuple[22], disabled=True,
                          default_text="12", disabled_readonly_background_color="#AAAAAA")],
                # (disabled=True, default_text="12") REMOVE AFTER STABLE RELASE
                [sg.Checkbox("Realtime simulation", key="-RTSIM-", enable_events=True, disabled=True,
                             # (disabled=True) REMOVE AFTER STABLE RELASE
                             tooltip="Enable realtime simulation only", auto_size_text=True,
                             checkbox_color=sg.DEFAULT_BACKGROUND_COLOR)]
            ])]
        ], vertical_alignment='Top', pad=((30, 0), (0, 0)))
        frame_5 = sg.Column([
            [sg.Column(temp1, visible=False, key='-Temp1-'),  # (visible=False) REMOVE AFTER STABLE RELASE
             sg.Column(temp2, visible=False, key='-Temp2-'),
             sg.Column(barmeter, visible=True, key='-Barometer-'),
             # (visible=True) CHANGE TO (visible=False) REMOVE AFTER STABLE RELASE
             sg.Column(tahmeter, visible=False, key='-Tahometer-'),
             sg.Column(pump, visible=False, key='-Pump-')],
            [sg.Frame("Simulation",
                      [[
                          sg.B("►", size=(5, 1), tooltip="Start simulation", disabled_button_color="black on white",
                               key="-PLAY-", enable_events=True, disabled=True),
                          sg.B("‖", size=(5, 1), tooltip="Stop simulation", disabled_button_color="black on white",
                               key="-PAUSE-", enable_events=True, disabled=True),
                          sg.B("►►", size=(5, 1), tooltip="Rewind simulation", disabled_button_color="black on white",
                               key="-REWIND-", enable_events=True)
                      ]]
                      )],
            [frame_3]
        ], vertical_alignment='Top', pad=((30, 0), (0, 0)))
        layout = [
            [
                sg.Column(windows_buttons)
            ],
            [frame_1, frame_5, frame_view_object]
        ]
        return sg.Window("Inputs", layout, size=(800, 500), finalize=True,
                         return_keyboard_events=True,
                         enable_close_attempted_event=True,
                         auto_size_text=False, auto_size_buttons=False)

    def controller_tab():
        windows_buttons = [
            [
                sg.B("Inputs [I]", key="-INPUTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Controller [C]", key="-CONTROLLER-", enable_events=True, disabled=True, size=(17, 1),
                     button_color='#AAAAAA'),
                sg.B("Output [O]", key="-OUTPUT-", enable_events=True, size=(17, 1), button_color='#AAAAAA',
                     disabled=True),  # (disabled=True) REMOVE AFTER STABLE RELASE
                sg.B("Charts [H]", key="-CHARTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Info [F1]", key="-INFO-", enable_events=True, size=(17, 1), button_color='#AAAAAA')
            ]
        ]
        frame3 = sg.Column([
            [sg.Frame("Controller", [
                [sg.Button(controllers_tuple[0], size=(5, 1), key='-' + controllers_tuple[0] + '_BUTTON-',
                           enable_events=True, disabled=True)],
                [sg.Button(controllers_tuple[1], size=(5, 1), key='-' + controllers_tuple[1] + '_BUTTON-',
                           enable_events=True)],
                [sg.Button(controllers_tuple[2], size=(5, 1), key='-' + controllers_tuple[2] + '_BUTTON-',
                           enable_events=True)],
                [sg.Button(controllers_tuple[3], size=(5, 1), key='-' + controllers_tuple[3] + '_BUTTON-',
                           enable_events=True)]
            ])],
            [sg.Text("W : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("000", size=(4, 1), pad=(0, 0), key=controller_io[0])],
            [sg.Text("S : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("Disabled", size=(6, 1), pad=(0, 0), key=controller_io[1])],
            [sg.Text("Z : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("Disabled", size=(6, 1), pad=(0, 0), key=controller_io[2])],
            [sg.Text("X : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("000", size=(4, 1), pad=(0, 0), key=controller_io[3])],
            [sg.Text("e : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("000", size=(4, 1), pad=(0, 0), key=controller_io[4])],
            [sg.Text("y : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("000", size=(4, 1), pad=(0, 0), key=controller_io[5])],
            [sg.Text("x : ", size=(5, 1), pad=((5, 0), (0, 0))),
             sg.Text("000", size=(4, 1), pad=(0, 0), key=controller_io[6])]
        ])
        frame1 = sg.Column([
            [sg.Frame("Select object", [[
                sg.Listbox(outputs_tuple, select_mode="LISTBOX_SELECT_MODE_SINGLE",
                           size=(26, 5), no_scrollbar=True, enable_events=True, key='-SELOBJ-', disabled=True,
                           # (disabled=True) REMOVE AFTER STABLE RELASE
                           tooltip="Select an object to enter its controller parameters",
                           default_values=[outputs_tuple[2]])
            ], [sg.Text("Selected : heater1", size=(15, 1))], [frame3]])]  # CHAGNE AFTER STABLE RELASE
        ], vertical_alignment='Top')
        frame2 = sg.Column([
            [sg.Frame("Wiew", [[
                sg.Image(filename=controllers_model[0], key="-CONTWIEW-")
            ]])]
        ])
        layout = [
            [
                sg.Column(windows_buttons)
            ],
            [frame1, frame2],
        ]
        return sg.Window("Controller", layout, size=(800, 500), finalize=True,
                         return_keyboard_events=True
                         , enable_close_attempted_event=True,
                         auto_size_text=False, auto_size_buttons=False)

    def output_tab():
        windows_buttons = [
            [
                sg.B("Inputs [I]", key="-INPUTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Controller [C]", key="-CONTROLLER-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Output [O]", key="-OUTPUT-", enable_events=True, disabled=True, size=(17, 1),
                     button_color='#AAAAAA'),
                sg.B("Charts [H]", key="-CHARTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Info [F1]", key="-INFO-", enable_events=True, size=(17, 1), button_color='#AAAAAA')
            ]
        ]
        frame_1 = sg.Column([
            [sg.Frame("Objects", [
                [sg.Text("Tank1 load : ", size=(9, 1)),
                 sg.Text("0000", key='-TANK1L-', size=(5, 1))],
                [sg.Text("Tank2 load : ", size=(9, 1)),
                 sg.Text("0000", key='-TANK2L-', size=(5, 1))],
                [sg.Text("Tank3 load : ", size=(9, 1)),
                 sg.Text("0000", key='-TANK3L-', size=(5, 1))],
                [sg.Text("Valve state : ", size=(9, 1)),
                 sg.Text("Closed", key='-VALVES-', size=(6, 1))],
                [sg.Text("Pump state : ", size=(9, 1)),
                 sg.Text("Disabled", key='-PUMPS-', size=(6, 1))]
            ])]
        ], vertical_alignment='t')
        frame_2 = sg.Column([
            [sg.Frame("Output terminal", [[
                sg.Output(size=(200, 500))
            ]])]
        ])
        layout = [
            [
                sg.Column(windows_buttons)
            ],
            [frame_1, frame_2]
        ]
        return sg.Window("Output", layout, size=(800, 500), finalize=True,
                         return_keyboard_events=True,
                         enable_close_attempted_event=True,
                         auto_size_text=False, auto_size_buttons=False)

    def charts_tab():
        windows_buttons = [
            [
                sg.B("Inputs [I]", key="-INPUTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Controller [C]", key="-CONTROLLER-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Output [O]", key="-OUTPUT-", enable_events=True, size=(17, 1), button_color='#AAAAAA',
                     disabled=True),  # (disabled=True) REMOVE AFTER STABLE RELASE
                sg.B("Charts [H]", key="-CHARTS-", enable_events=True, disabled=True, size=(17, 1),
                     button_color='#AAAAAA'),
                sg.B("Info [F1]", key="-INFO-", enable_events=True, size=(17, 1), button_color='#AAAAAA')
            ]
        ]
        graph1 = sg.Graph(canvas_size=(500, 190),
                          graph_bottom_left=(0, 0),
                          graph_top_right=(1400, 1400),
                          background_color='#EEEEEE',
                          key='graph1')
        graph2 = sg.Graph(canvas_size=(500, 190),
                          graph_bottom_left=(0, 0),
                          graph_top_right=(1400, 1400),
                          background_color='#EEEEEE',
                          key='graph2')
        graph1_frame = sg.Frame("Chart 1", [[graph1]])
        graph2_frame = sg.Frame("Chart 2", [[graph2]])
        graph_col = sg.Column([[graph1_frame], [graph2_frame]])
        frame1 = sg.Column([
            [sg.Frame("Select object", [[
                sg.Listbox(outputs_tuple, select_mode="LISTBOX_SELECT_MODE_SINGLE",
                           size=(26, 5), no_scrollbar=True, enable_events=True, key='-SELOBJ-', disabled=True,
                           # (disabled=True) REMOVE AFTER STABLE RELASE
                           tooltip="Select an object to view its characteristics", default_values=[outputs_tuple[2]])
            ], [sg.Text("Selected : heater1", size=(15, 1))], [
                sg.B("UPDATE", key="-UPD-", enable_events=True, size=(17, 1))
            ]])]  # CHANGE AFTER STABLE RELASE
        ], vertical_alignment='Top')
        layout = [
            [
                sg.Column(windows_buttons)
            ],
            [graph_col, frame1]
        ]
        return sg.Window("Charts", layout, size=(800, 500), finalize=True,
                         return_keyboard_events=True,
                         enable_close_attempted_event=True,
                         auto_size_text=False, auto_size_buttons=False)

    def info_tab():
        windows_buttons = [
            [
                sg.B("Inputs [I]", key="-INPUTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Controller [C]", key="-CONTROLLER-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Output [O]", key="-OUTPUT-", enable_events=True, size=(17, 1), button_color='#AAAAAA',
                     disabled=True),  # (disabled=True) REMOVE AFTER STABLE RELASE
                sg.B("Charts [H]", key="-CHARTS-", enable_events=True, size=(17, 1), button_color='#AAAAAA'),
                sg.B("Info [F1]", key="-INFO-", enable_events=True, disabled=True, size=(17, 1), button_color='#AAAAAA')
            ]
        ]

        frame_authors_relase = sg.Column([
            [sg.Frame("Author:",
                      [[sg.Text("\
                Concept, design and code : \n\
                    Mateusz Ferenc",
                                size=(32, 10))]])
             ],
            [sg.Frame("Relase info:",
                      [[sg.Text("\
                Date : \n\
                    " + relase_date + "\n\
                Version : \n\
                    " + version + "\n\
                Developer: \n\
                    M. Ferenc",
                                size=(32, 11))]])
             ]
        ])

        frame_openfiles = sg.Column([  # TODO
            [sg.Frame("Select files:",
                      [[
                          sg.Text("None", size=(64, 23))
                      ]]
                      )]
        ])

        layout = [
            [
                sg.Column(windows_buttons)
            ],
            [frame_authors_relase, frame_openfiles]
        ]

        return sg.Window("Info", layout, size=(800, 500), finalize=True,
                         return_keyboard_events=True,
                         enable_close_attempted_event=True,
                         auto_size_text=False, auto_size_buttons=False)

    def wiew_schematic():
        return sg.Window("Schematic", [[sg.Image(filename=schematic_file)]], finalize=True)

    controller = controller_tab()
    output = output_tab()
    charts = charts_tab()
    info = info_tab()
    controller.hide()
    output.hide()
    charts.hide()
    charts.hide()
    info.hide()
    inputs, schematic = inputs_tab(), None

    graph1 = charts['graph1']
    graph2 = charts['graph2']

    # Checks inputs for valid format, (min, max) - optional to check if input is in valid range, float_type - checks input of float type
    # time_type - check input of time type, test_minmax - check input against lower value than minmax
    def is_valid(input, format, min=None, max=None, float_type=False, time_type=False, test_minmax=False, minmax=None):
        test1 = re.search(format, input)  # REGEXP search for string by given filter (format)
        try:
            test1.group(0)  # Try to get value from first group, if input is not as format it casues exception
        except:
            return False  # input is not valid
        if min is not None and max is not None:  # Do if min and max values are given
            if float_type:  # Do if float type flag is raised
                if min <= float(input) and max >= float(input):  # Check input to be in range of min and max
                    if test_minmax:  # Test if input (max) value is not lower than minmax
                        if minmax == '':
                            return False
                        if float(input) < float(minmax):
                            return False  # input lower than minmax
                        else:
                            return True  # input in range min, max and higher than minmax
                    else:
                        return True  # input in range min, max
                else:
                    return False  # input beyond range min, max
            elif min <= int(input) and max >= int(input):  # Check input to be in range of min and max
                if test_minmax:  # Test if input (max) value is not lower than minmax
                    if minmax == '':
                        return False
                    if int(input) < int(minmax):
                        return False  # input lower than minmax
                    else:
                        return True  # input in range min, max and higher than minmax
                else:
                    return True  # input in range min, max
            else:
                return False  # input beyond range min, max
        else:  # Do if min and max are not specified
            if time_type:  # Do if time type flag is raised
                time = re.search("([0-9]{1,2}):([0-9]{2}):([0-9]{2})",
                                 input)  # Divide time (input) to hours, minutes and seconds
                if 1 <= int(time.group(1)) and 99 >= int(time.group(1)):  # Check if hours are in range of 1 to 99
                    if 0 <= int(time.group(2)) and 59 >= int(time.group(2)):  # Check if minutes are in range of 1 to 59
                        if 0 <= int(time.group(3)) and 59 >= int(
                                time.group(3)):  # Check if seconds are in range of 1 to 59
                            return True
                        else:
                            return False
                    else:
                        if 0 <= int(time.group(3)) and 59 >= int(time.group(3)):
                            return True
                        else:
                            return False
                else:
                    if 1 <= int(time.group(2)) and 59 >= int(time.group(2)):
                        if 0 <= int(time.group(3)) and 59 >= int(time.group(3)):
                            return True
                        else:
                            return False
                    else:
                        if 1 <= int(time.group(3)) and 59 >= int(time.group(3)):
                            return True
                        else:
                            return False
            else:
                return True

    # Returns None if is_valid or noise_dis is True, otherwise return inputs_t
    def return_bad(is_valid, inputs_t, noise_dis=False):
        return None if (is_valid or noise_dis) else inputs_t

    # Returns tuple with values depending on corectness of inputs
    def test_inputs():  # IMPORTANT! Remember the correct order!!!
        bad = []
        input1 = []
        global noise_disabled
        input1.append(inputs[parameters_tuple[0]].Get())
        bad.append(return_bad(is_valid(input1[0], formats_tuple[0], 20.0, 150.0, True), parameters_tuple[0],
                              True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[1]].Get())
        bad.append(return_bad(
            is_valid(input1[1], formats_tuple[0], 20.0, 150.0, float_type=True, test_minmax=True, minmax=input1[0]),
            parameters_tuple[1], True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[2]].Get())
        bad.append(return_bad(is_valid(input1[2], formats_tuple[3], 0, 100), parameters_tuple[2],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[3]].Get())
        bad.append(return_bad(is_valid(input1[3], formats_tuple[3], 0, 100), parameters_tuple[3],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[4]].Get())
        bad.append(return_bad(is_valid(input1[4], formats_tuple[0], 20.0, 150.0, float_type=True), parameters_tuple[4],
                              True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[5]].Get())
        bad.append(return_bad(
            is_valid(input1[5], formats_tuple[0], 20.0, 150.0, float_type=True, test_minmax=True, minmax=input1[4]),
            parameters_tuple[5], True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[6]].Get())
        bad.append(return_bad(is_valid(input1[6], formats_tuple[3], 0, 100), parameters_tuple[6],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[7]].Get())
        bad.append(return_bad(is_valid(input1[7], formats_tuple[3], 0, 100), parameters_tuple[7],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[8]].Get())
        bad.append(return_bad(is_valid(input1[8], formats_tuple[1], 800, 1500), parameters_tuple[8]))
        input1.append(inputs[parameters_tuple[9]].Get())
        bad.append(return_bad(is_valid(input1[9], formats_tuple[1], 800, 1500, test_minmax=True, minmax=input1[8]),
                              parameters_tuple[9]))
        input1.append(inputs[parameters_tuple[10]].Get())
        bad.append(return_bad(is_valid(input1[10], formats_tuple[3], 0, 100), parameters_tuple[10], noise_disabled))
        input1.append(inputs[parameters_tuple[11]].Get())
        bad.append(return_bad(is_valid(input1[11], formats_tuple[3], 0, 100), parameters_tuple[11], noise_disabled))
        input1.append(inputs[parameters_tuple[12]].Get())
        bad.append(return_bad(is_valid(input1[12], formats_tuple[2], 10, 80), parameters_tuple[12],
                              True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[13]].Get())
        bad.append(return_bad(is_valid(input1[13], formats_tuple[2], 10, 80, test_minmax=True, minmax=input1[12]),
                              parameters_tuple[13], True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[14]].Get())
        bad.append(return_bad(is_valid(input1[14], formats_tuple[3], 0, 100), parameters_tuple[14],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[15]].Get())
        bad.append(return_bad(is_valid(input1[15], formats_tuple[3], 0, 100), parameters_tuple[15],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[16]].Get())
        bad.append(return_bad(is_valid(input1[16], formats_tuple[3], 0, 100), parameters_tuple[16],
                              True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[17]].Get())
        bad.append(return_bad(is_valid(input1[17], formats_tuple[3], 0, 100), parameters_tuple[17],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[18]].Get())
        bad.append(return_bad(is_valid(input1[18], formats_tuple[3], 0, 100), parameters_tuple[18],
                              True))  # (, True) CHANGE TO (, noise_disabled) AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[19]].Get())
        bad.append(return_bad(is_valid(input1[19], formats_tuple[1], 100, 9999), parameters_tuple[19]))
        input1.append(inputs[parameters_tuple[20]].Get())
        bad.append(return_bad(is_valid(input1[20], formats_tuple[5], time_type=True), parameters_tuple[20]))
        input1.append(inputs[parameters_tuple[21]].Get())
        bad.append(return_bad(is_valid(input1[21], formats_tuple[3], 1, 100), parameters_tuple[21]))
        input1.append(inputs[parameters_tuple[22]].Get())
        bad.append(return_bad(is_valid(input1[22], formats_tuple[6], 8, 18), parameters_tuple[22],
                              True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[23]].Get())
        bad.append(return_bad(is_valid(input1[23], formats_tuple[1], 100, 9999, test_minmax=True, minmax=input1[19]),
                              parameters_tuple[23]))
        input1.append(inputs[parameters_tuple[24]].Get())
        bad.append(return_bad(is_valid(input1[24], formats_tuple[1], 100, 9999, test_minmax=True, minmax=input1[19]),
                              parameters_tuple[24], True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[25]].Get())
        bad.append(return_bad(is_valid(input1[25], formats_tuple[1], 100, 9999, test_minmax=True, minmax=input1[19]),
                              parameters_tuple[25], True))  # (, True) REMOVE AFTER STABLE RELASE
        input1.append(inputs[parameters_tuple[26]].Get())
        bad.append(return_bad(is_valid(input1[26], formats_tuple[3], 0, 100), parameters_tuple[26]))
        input1.append(inputs[parameters_tuple[27]].Get())
        bad.append(return_bad(is_valid(input1[27], formats_tuple[3], 0, 100), parameters_tuple[27], noise_disabled))
        input1.append(inputs[parameters_tuple[28]].Get())
        bad.append(return_bad(is_valid(input1[28], formats_tuple[3], 0, 100), parameters_tuple[28], noise_disabled))
        if any(bad):
            return True, bad
        else:
            return False, input1

    # Marks red invalid fields
    def select_invalid(input):  # Because this definition needs correct order of inputs
        for x, y in zip(parameters_tuple, range(len(parameters_tuple) + 1)):
            inputs[x].update(background_color=sg.theme_background_color() if input[y] is None else "#FF0000")

    # Disables and changes colors of fields for simulation
    def disable_inputs():
        update_values = True
        inputs['-ENOISE-'].update(disabled=True)
        inputs['-RTSIM-'].update(disabled=True)
        for x in parameters_tuple:
            inputs[x].update(background_color="#AAAAAA", text_color="#000000", disabled=True)

    # Enables and changes colors of fields for simulation
    def enable_inputs():
        update_values = False
        inputs['-ENOISE-'].update(disabled=False)
        inputs['-RTSIM-'].update(disabled=False)
        for x in parameters_tuple:
            inputs[x].update(background_color=sg.theme_background_color(), text_color="#FFFFFF",
                             disabled=False if 'None' in str(
                                 re.search("-[0-9]+(N)", x)) else True if noise_disabled else False)

    def Draw_chart(g_obj, OX_length, OY_length, max_OX, max_OY, OX_marker_freq, OY_marker_freq,
                   OX_offset=0, OY_offset=0, OX_format='{:02d}.{:02d}', OY_format='{:02d}.{:02d}', OX_name='X',
                   OY_name='Y',
                   OX_color='black', OY_color='black', OX_mark_1=1, OX_mark_2=1, OY_mark_1=1, OY_mark_2=1):
        g_obj.draw_line((0 + OX_offset, 0 + OY_offset), (OX_length + 60 + OX_offset, 0 + OY_offset))
        g_obj.draw_line((0 + OX_offset, 0 + OY_offset), (0 + OX_offset, OY_length + 60 + OY_offset))
        g_obj.draw_line((-8 + OX_offset, -8 + OY_offset), (0 + OX_offset, 0 + OY_offset), color='red')
        g_obj.draw_text("0", (-25 + OX_offset, -60 + OY_offset), color='black')
        Draw_arrow(g_obj, OX_length + 60 + OX_offset, 0 + OY_offset, 40, 20, dir='Right')
        Draw_arrow(g_obj, 0 + OX_offset, OY_length + 60 + OY_offset, 20, 40)

        x1 = max_OX / OX_marker_freq
        for x in range(0, OX_length + 1, int(round(OX_length / 10))):
            if x != 0:
                g_obj.draw_line((x + OX_offset, -15 + OY_offset), (x + OX_offset, 0 + OY_offset), color='red')
                g_obj.draw_text(OX_format.format((x1 / OX_mark_1), (x1 / OX_mark_2)), (x + OX_offset, -60 + OY_offset),
                                color='black')
                Draw_dotted_line(g_obj, x + OX_offset, 0 + OY_offset, OY_length, dir='Vertical', dot_length=10,
                                 dot_distance=20, _color='#CCCCCC')
                x1 += max_OX / OX_marker_freq
        g_obj.draw_text(OX_name, (OX_length + 100 + OX_offset, 0 + OY_offset), color=OX_color)

        y1 = max_OY / OY_marker_freq
        for y in range(0, OY_length + 1, int(round(OY_length / 10))):
            if y != 0:
                g_obj.draw_line((-8 + OX_offset, y + OY_offset), (0 + OX_offset, y + OY_offset), color='red')
                g_obj.draw_text(OY_format.format((y1 / OY_mark_1), (y1 / OY_mark_2)), (-65 + OX_offset, y + OY_offset),
                                color='black')
                Draw_dotted_line(g_obj, 0 + OX_offset, y + OY_offset, OX_length, dir='Horizontal', dot_length=10,
                                 dot_distance=20, _color='#CCCCCC')
                y1 += max_OY / OY_marker_freq
        g_obj.draw_text(OY_name, (0 + OX_offset, OY_length + 120 + OY_offset), color=OY_color)

    def Draw_from_values(g_obj, x, y, sample_rate, OX_max, OY_vals, x_name='x', y_name='y', draw_color='black',
                         x_offset=0, y_offset=0, ox_length=1000, oy_length=1100):
        g_obj.erase()
        Draw_chart(g_obj, ox_length, oy_length, OX_max, max(OY_vals), 10, 10, OX_offset=x_offset, OY_offset=y_offset,
                   OX_format='{:n}s', OY_format='{:.1f}', OX_name=x_name, OY_name=y_name, OX_color='black',
                   OY_color=draw_color,
                   OX_mark_1=1, OX_mark_2=1, OY_mark_1=1, OY_mark_2=1)
        Draw_lines_by_step(g_obj, x, y, OY_vals, ((len(OY_vals) / OX_max)), ox_length, oy_length, (OX_max),
                           max(OY_vals), x_off=x_offset, y_off=y_offset, _color=draw_color)

    def Draw_lines_by_step(g_obj, x, y, values, step, x_length, y_length, x_max, y_max, x_off=0, y_off=0,
                           _color='black'):
        last_x, last_y = x + 1, int(round((y_length * values[0]) / y_max))
        _x = int((x_length * step) / (x_max))
        s = _x
        for v in values:
            if v == values[0]:
                continue
            _y = int(round((y_length * v) / y_max))
            g_obj.draw_line((last_x + x_off, last_y + y_off), (s + x_off, _y + y_off), color=_color)
            last_x, last_y = s, _y
            s += _x

    def Draw_arrow(g_obj, x, y, width, length, dir='Top', c='black'):
        if dir == 'Top':
            y1 = y - length
            x1 = x + int(round(width / 2))
            x2 = x - int(round(width / 2))
            g_obj.draw_line((x, y), (x1, y1), color=c)
            g_obj.draw_line((x, y), (x2, y1), color=c)
        elif dir == 'Bottom':
            y1 = y + length
            x1 = x + int(round(width / 2))
            x2 = x - int(round(width / 2))
            g_obj.draw_line((x, y), (x1, y1), color=c)
            g_obj.draw_line((x, y), (x2, y1), color=c)
        elif dir == 'Right':
            x1 = x - length
            y1 = y + int(round(width / 2))
            y2 = y - int(round(width / 2))
            g_obj.draw_line((x, y), (x1, y1), color=c)
            g_obj.draw_line((x, y), (x1, y2), color=c)
        elif dir == 'Left':
            x1 = x + length
            y1 = y + int(round(width / 2))
            y2 = y - int(round(width / 2))
            g_obj.draw_line((x, y), (x1, y1), color=c)
            g_obj.draw_line((x, y), (x1, y2), color=c)

    def Draw_dotted_line(g_obj, x, y, length, dir='Vertical', dot_length=1, dot_distance=10, _color='black'):
        spaces = dots = 0
        for l in range(length + 1):
            if spaces != dot_distance:
                spaces += 1
            else:
                if dots != dot_length:
                    dots += 1
                    g_obj.draw_line((x + l if dir == 'Horizontal' else x, y + l if dir == 'Vertical' else y), (
                    (x + l if dir == 'Horizontal' else x) + (1 if dir == 'Horizontal' else 0),
                    (y + l if dir == 'Vertical' else y) + (1 if dir == 'Vertical' else 0)), color=_color)
                else:
                    spaces = dots = 0

    while True:
        window, event, values = sg.read_all_windows(timeout=500)
        if event == "WINDOW_CLOSE_ATTEMPTED_EVENT" or event == sg.WIN_CLOSED:
            global schematic_opened
            if schematic_opened:
                schematic_opened = False
                schematic.close()
            else:
                break
                # answer = sg.popup_yes_no("Are you sure want to exit?", title="Please specify")
                # if answer == 'Yes':
                #    sg.popup_notify("Ok, bye then!", title='Bye', display_duration_in_ms = 200)
                #    break
                # else:
                #    sg.popup_no_buttons("Good!", title='Thanks', auto_close = True, non_blocking=True, auto_close_duration=1)"""
                # (#) REMOVE AFTER STABLE RELASE

        if window == inputs:
            if event == '-SELINPUT-' and len(values['-SELINPUT-']):
                val = re.search("\[('(.+)')\]", str(values['-SELINPUT-']))
                inputs[in_last_con].update(visible=False)
                val1 = re.sub("(^)(.+)($)", "\\1-\\2\\3-", str(val.group(2)))
                in_last_con = val1.replace(" ", "")
                inputs[in_last_con].update(visible=True)

            if event == '-ENOISE-':
                val = re.search("True|False", str(values['-ENOISE-']))
                if val.group(0) == 'True':
                    noise_disabled = False
                    for o in parameters_tuple:
                        if o in (parameters_tuple[2], parameters_tuple[3], parameters_tuple[6], parameters_tuple[7],
                                 parameters_tuple[14], parameters_tuple[15], parameters_tuple[17],
                                 parameters_tuple[18]):  # REMOVE AFTER STABLE RELASE
                            continue
                        if re.search("-[0-9]+(N)", o):
                            inputs[o].update(disabled=False)
                else:
                    noise_disabled = True
                    for o in parameters_tuple:
                        if o in (parameters_tuple[2], parameters_tuple[3], parameters_tuple[6], parameters_tuple[7],
                                 parameters_tuple[14], parameters_tuple[15], parameters_tuple[17],
                                 parameters_tuple[18]):  # REMOVE AFTER STABLE RELASE
                            continue
                        if re.search("-[0-9]+(N)", o):
                            inputs[o].update(disabled=True)

            if event == '-SIMIMG-' and schematic_opened == False:
                schematic_opened = True
                schematic = wiew_schematic()

            inputs['-PLAY-'].update(disabled=False if values['-RTSIM-'] is True else True)
            inputs['-PAUSE-'].update(disabled=False if values['-RTSIM-'] is True else True)
            inputs['-REWIND-'].update(disabled=False if values['-RTSIM-'] or sim_rewind is False else True)
            inputs['-RTADJ-'].update(visible=False if values['-RTSIM-'] is False else True)

            if event == '-PLAY-' and sim_paused and not sim_rewind:
                is_bad, bad = test_inputs()
                if is_bad:
                    select_invalid(bad)
                else:
                    sim_paused = False
                    disable_inputs()
                    inputs['-PLAY-'].update(disabled=True)
                    inputs['-PAUSE-'].update(disabled=False)

            if event == '-PAUSE-' and not sim_paused and not sim_rewind:
                sim_paused = True
                enable_inputs()
                inputs['-PAUSE-'].update(disabled=True)
                inputs['-PLAY-'].update(disabled=False)

            if event == '-REWIND-' and not sim_rewind:
                is_bad, bad = test_inputs()
                if is_bad:
                    select_invalid(bad)
                else:
                    sim_rewind = True
                    disable_inputs()
                    for x in ['-PLAY-', '-PAUSE-', '-REWIND-']:
                        inputs[x].update(disabled=True)

            if event == '-CONTROLLER-' or event == 'c':
                inputs.hide()
                controller.un_hide()
            if event == '-OUTPUT-' or event == 'o':
                pass  # REMOVE AFTER STABLE RELASE
                # inputs.hide()
                # output.un_hide()
            if event == '-CHARTS-' or event == 'h':
                inputs.hide()
                charts.un_hide()
            if event == '-INFO-' or event == 'F1:112':
                inputs.hide()
                info.un_hide()

        if window == controller:
            if re.search("-[A-Z]{1,3}_BUTTON-", event):
                # r = re.search("-([A-Z]{1,3})_BUTTON-", event)
                for o in controllers_tuple:
                    controller['-' + o + '_BUTTON-'].update(disabled=True if '-' + o + '_BUTTON-' == event else False)
                    if '-' + o + '_BUTTON-' == event:
                        actual_controller = o
                c = 0
                for pic in controllers_model:
                    if re.search(str("./Graph/" + actual_object + "_" + actual_controller + "_s.png"), pic):
                        controller['-CONTWIEW-'].update(filename=controllers_model[c])
                    c += 1

            if event == '-SELOBJ-' and len(values['-SELOBJ-']):
                val = re.search("\['(.+)'\]", str(values['-SELOBJ-']))
                actual_object = val.group(1)
                c = 0
                for pic in controllers_model:
                    if re.search(str("./Graph/" + actual_object + "_" + actual_controller + "_s.png"), pic):
                        controller['-CONTWIEW-'].update(filename=controllers_model[c])
                    c += 1
                # for c in controller_io:
                #    controller[c].update(text=)

            if update_values == True:
                update_values = False
                for c in controller_io:
                    pass  # controller[o].update(text=)

            if event == '-INPUTS-' or event == 'i':
                controller.hide()
                inputs.un_hide()
            if event == '-OUTPUT-' or event == 'o':
                pass  # REMOVE AFTER STABLE RELASE
                # controller.hide()
                # output.un_hide()
            if event == '-CHARTS-' or event == 'h':
                controller.hide()
                charts.un_hide()
            if event == '-INFO-' or event == 'F1:112':
                controller.hide()
                info.un_hide()

        if window == output:
            # print(str("Howdy" + event), file = sys.stdout)
            if event == '-INPUTS-' or event == 'i':
                output.hide()
                inputs.un_hide()
            if event == '-CONTROLLER-' or event == 'c':
                output.hide()
                controller.un_hide()
            if event == '-CHARTS-' or event == 'h':
                output.hide()
                charts.un_hide()
            if event == '-INFO-' or event == 'F1:112':
                output.hide()
                info.un_hide()

        if window == charts:
            if event == '-UPD-':
                sr1 = 1
                sr2 = 1
                r1 = 10  # random.randrange(1, 1000)
                var1 = []
                ra1 = (r1 * sr1) + 1
                for v in range(ra1):
                    var1.append(random.randrange(0, 101))
                r2 = 10  # random.randrange(1, 1000)
                var2 = []
                ra2 = (r2 * sr2) + 1
                for v in range(ra2):
                    var2.append(random.randrange(800, 1501))
                Draw_from_values(graph1, 0, 0, sample_rate=sr1, OX_max=r1, OY_vals=var1, x_name='Time', y_name='Value',
                                 draw_color='#0000FF', x_offset=110, y_offset=110)
                Draw_from_values(graph2, 0, 0, sample_rate=sr2, OX_max=r2, OY_vals=var2, x_name='Time',
                                 y_name='Pressure', draw_color='#FF0000', x_offset=110, y_offset=110)
            if event == '-INPUTS-' or event == 'i':
                charts.hide()
                inputs.un_hide()
            if event == '-CONTROLLER-' or event == 'c':
                charts.hide()
                controller.un_hide()
            if event == '-OUTPUT-' or event == 'o':
                pass  # REMOVE AFTER STABLE RELASE
                # charts.hide()
                # output.un_hide()
            if event == '-INFO-' or event == 'F1:112':
                charts.hide()
                info.un_hide()

        if window == info:
            if event == '-INPUTS-' or event == 'i':
                info.hide()
                inputs.un_hide()
            if event == '-CONTROLLER-' or event == 'c':
                info.hide()
                controller.un_hide()
            if event == '-OUTPUT-' or event == 'o':
                pass  # REMOVE AFTER STABLE RELASE
                # info.hide()
                # output.un_hide()
            if event == '-CHARTS-' or event == 'c':
                info.hide()
                charts.un_hide()

    window.close()


def DSP(dsp):
    t1 = ms = 0
    while True:
        if dsp.Get_time_update_flag():
            hours, minutes, seconds = dsp.Get_time()
        if dsp.Is_enabled() and not dsp.Get_time_update_flag():
            t = time.time()
            if t - t1 >= 1.0:
                ms = 0
                if seconds > 0:
                    seconds -= 1
                elif minutes > 0:
                    seconds = 59
                    minutes -= 1
                elif hours > 0:
                    minutes = 59
                    seconds = 59
                    hours -= 1
                else:
                    dsp.Disable()
                t1 = time.time()
            else:
                ms += 1


def down_counter(cd_timer):
    sg.theme('DarkGrey8')
    timer = sg.Window("Timer", [[sg.Text(size=(95, 75), font=('Helvetica', 40), key='TIMER_COUNTER')]], size=(250, 80),
                      finalize=True, location=(50, 50), disable_close=True, element_justification="center",
                      keep_on_top=True, disable_minimize=True)
    t1 = 0
    timer.hide()
    while True:
        event, values = timer.read(timeout=1)
        if cd_timer.Get_update_flag():
            hours, minutes, seconds = cd_timer.Get_time()
            timer['TIMER_COUNTER'].update('{:02d}:{:02d}:{:02d}'.format(hours, minutes, seconds))
            timer.refresh()
            timer.un_hide()
        if cd_timer.Is_enabled() and not cd_timer.Get_update_flag():
            t = time.time()
            if t - t1 >= 1.0:
                timer['TIMER_COUNTER'].update('{:02d}:{:02d}:{:02d}'.format(hours, minutes, seconds))
                timer.refresh()
                if seconds > 0:
                    seconds -= 1
                elif minutes > 0:
                    seconds = 59
                    minutes -= 1
                elif hours > 0:
                    minutes = 59
                    seconds = 59
                    hours -= 1
                else:
                    cd_timer.Disable()
                    timer.hide()
                t1 = time.time()

    timer.close()


if __name__ == '__main__':
    BaseManager.register('Countdown_timer', Countdown_timer)
    manager = BaseManager()
    manager.start()
    ct = manager.Countdown_timer()
    BaseManager.register('DSP_container', DSP_container)
    manager2 = BaseManager()
    manager2.start()
    dsp = manager2.DSP_container()
    t = Process(target=down_counter, args=(ct,))
    t.Deamon = True
    d = Process(target=DSP, args=(dsp,))
    d.Deamon = True
    m = Process(target=main, args=(ct, dsp))

    m.start()
    t.start()
    d.start()
    m.join()
    t.terminate()
    d.terminate()
