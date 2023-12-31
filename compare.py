from vpython import (
    sphere,
    rate,
    vector,
    color,
    scene,
    gcurve,
    cylinder,
    slider,
    wtext,
    checkbox,
    sin,
    graph,
    radio,
)
from controllers import (
    Controller,
    PIDController,
    BangBangController,
    POnlyController,
    FuzzyLogicController,
    LQRController,
)

from params import dt, n_fft
import numpy as np
from enum import Enum
import random


class FunctionType(Enum):
    SINE = 1
    SQUARE = 2
    SAWTOOTH = 3


class FunctionGenerator:
    def __init__(
        self,
        amplitude=5.0,
        frequency=1,
        offset=5.0,
        func_type=FunctionType.SINE,
        noise_level=0.2,
    ):
        self.amplitude = amplitude
        self.frequency = frequency
        self.offset = offset
        self.func_type = func_type
        self.noise_level = noise_level

    def set_params(self, **kwargs):
        self.amplitude = kwargs.get("amplitude", self.amplitude)
        self.frequency = kwargs.get("frequency", self.frequency)
        self.offset = kwargs.get("offset", self.offset)
        self.func_type = kwargs.get("func_type", self.func_type)
        self.noise_level = kwargs.get("noise_level", self.noise_level)

    def add_noise(self, value):
        return value + random.uniform(-self.noise_level, self.noise_level)

    def __call__(self, t):
        value = self.offset
        if self.func_type == FunctionType.SINE:
            value += self.amplitude * sin(self.frequency * t)
        elif self.func_type == FunctionType.SQUARE:
            value += self.amplitude * (1 if sin(self.frequency * t) >= 0 else -1)
        elif self.func_type == FunctionType.SAWTOOTH:
            value += self.amplitude * (
                2 * (t * self.frequency - int(t * self.frequency + 0.5))
            )

        return self.add_noise(value)


function_generator = FunctionGenerator()


def target_function(t: float) -> float:
    """Calculate the target height based on time."""
    return function_generator(t)


class ControlVisualizer:
    def __init__(
        self,
        graph1: gcurve,
        graph2: gcurve,
        graph3: gcurve,
        graph4: gcurve,
        ball: sphere,
        controller: Controller,
        checkbox: checkbox,
    ) -> None:
        self.graph1 = graph1
        self.graph2 = graph2
        self.graph3 = graph3
        self.graph4 = graph4
        self.ball = ball
        self.controller = controller
        self.checkbox = checkbox
        self.errors = []
        self.last_update_time = (
            0  # Added this line to keep track of the last update time
        )

    def update_ball(self, target_height: float, t: float) -> None:
        """Update the position of the ball based on the controller."""
        error = target_height - self.ball.pos.y
        self.errors.append(error)
        output = self.controller.update(error)
        self.controller.tune(error)
        self.ball.pos.y += output * dt
        if self.checkbox.checked:
            self.graph1.plot(t, error)
            self.graph3.plot(t, np.max(self.errors) - target_height)  # Overshoot
            self.graph4.plot(
                t, np.abs(self.errors[-1] - target_height)
            )  # Steady state error
        # move ball forward simulating time
        self.ball.pos.z += dt

        if t > 0 and self.checkbox.checked and t - self.last_update_time >= 1.0:
            self.graph2.delete()
            fft_values = np.abs(np.fft.fft(self.errors))
            freqs = np.fft.fftfreq(len(fft_values), dt)
            for freq, fft_val in zip(freqs, fft_values):
                if freq > 0:
                    self.graph2.plot(freq, np.log10(fft_val), fast=True)
            self.last_update_time = t  # Update the last update time

        if len(self.errors) > n_fft:
            self.errors.pop(0)

    def hide_ball(self):
        """Hide the ball by setting its opacity to 0."""
        self.ball.opacity = 0


# Initialize graphs
graph(scroll=True, fast=True, xmin=0, xmax=10)
target_graph = gcurve(color=color.yellow, label="Target Height")
pid_error_graph = gcurve(color=color.red, label="PID Error")
bang_error_graph = gcurve(color=color.green, label="Bang-Bang Error")
p_error_graph = gcurve(color=color.blue, label="P-Only Error")
fuzzy_error_graph = gcurve(color=color.purple, label="Fuzzy Logic Error")
lqr_error_graph = gcurve(color=color.magenta, label="LQR Error")

# Initialize frequency domain graphs
graph2 = graph(scroll=False, fast=True, xmin=0, xmax=50)
target_graph2 = gcurve(color=color.yellow, label="Target Height")
pid_error_graph2 = gcurve(color=color.red, label="PID Error")
bang_error_graph2 = gcurve(color=color.green, label="Bang-Bang Error")
p_error_graph2 = gcurve(color=color.blue, label="P-Only Error")
fuzzy_error_graph2 = gcurve(color=color.purple, label="Fuzzy Logic Error")
lqr_error_graph2 = gcurve(color=color.magenta, label="LQR Error")

# Initialize overshoot graphs
graph3 = graph(scroll=True, fast=True, xmin=0, xmax=10)
pid_error_graph3 = gcurve(color=color.red, label="PID Overshoot")
bang_error_graph3 = gcurve(color=color.green, label="Bang-Bang Overshoot")
p_error_graph3 = gcurve(color=color.blue, label="P-Only Overshoot")
fuzzy_error_graph3 = gcurve(color=color.purple, label="Fuzzy Logic Overshoot")
lqr_error_graph3 = gcurve(color=color.magenta, label="LQR Overshoot")

# Initialize steady state error graphs
graph4 = graph(scroll=True, fast=True, xmin=0, xmax=10)
pid_error_graph4 = gcurve(color=color.red, label="PID steady state error")
bang_error_graph4 = gcurve(color=color.green, label="Bang-Bang steady state error")
p_error_graph4 = gcurve(color=color.blue, label="P-Only steady state error")
fuzzy_error_graph4 = gcurve(color=color.purple, label="Fuzzy Logic steady state error")
lqr_error_graph4 = gcurve(color=color.magenta, label="LQR steady state error")

pid_ball = sphere(
    pos=vector(-5, 0, 0),
    radius=1,
    color=color.red,
    make_trail=True,
    retain=1000,
    trail_radius=0.1,
)
bang_ball = sphere(
    pos=vector(0, 0, 0),
    radius=1,
    color=color.green,
    make_trail=True,
    retain=1000,
    trail_radius=0.1,
)
p_only_ball = sphere(
    pos=vector(5, 0, 0),
    radius=1,
    color=color.blue,
    make_trail=True,
    retain=1000,
    trail_radius=0.1,
)
fuzzy_ball = sphere(
    pos=vector(10, 0, 0),
    radius=1,
    color=color.purple,
    make_trail=True,
    retain=1000,
    trail_radius=0.1,
)
lqr_ball = sphere(
    pos=vector(15, 0, 0),
    radius=1,
    color=color.magenta,
    make_trail=True,
    retain=1000,
    trail_radius=0.1,
)


target = cylinder(
    pos=vector(-5, target_function(0), 0),
    axis=vector(20, 0, 0),
    radius=0.1,
    color=color.yellow,
)
scene.range = 10

# Initialize controllers
pid_controller = PIDController(Kp=1.0, Ki=0.1, Kd=dt)
bang_controller = BangBangController(delta=1)
p_only_controller = POnlyController(Kp=1.0)
fuzzy_controller = FuzzyLogicController(delta=100)
# Parameters for LQR (A and B are from the state-space representation, Q and R are weights)
A = 1  # For a simple integrator
B = 1  # For a simple integrator
Q = 1  # Weight for the state
R = 0.1  # Weight for the control input
lqr_controller = LQRController(A, B, Q, R)


# Slider callback functions


def set_Kp(slider):
    global pid_controller
    pid_controller.Kp = slider.value


def set_Ki(slider):
    global pid_controller
    pid_controller.Ki = slider.value


def set_Kd(slider):
    global pid_controller
    pid_controller.Kd = slider.value


# Initialize sliders and labels

wtext(text=" Kp")
Kp_slider = slider(min=0, max=5, value=1.0, length=300, bind=set_Kp)
wtext(text=" Ki")
Ki_slider = slider(min=0, max=1, value=0.1, length=300, bind=set_Ki)
wtext(text=" Kd")
Kd_slider = slider(min=0, max=0.1, value=dt, length=300, bind=set_Kd)


# Callback functions for sliders
def set_A(slider):
    global lqr_controller
    lqr_controller.A = slider.value
    lqr_controller.K = lqr_controller.calculate_gain(
        lqr_controller.A, lqr_controller.B, lqr_controller.Q, lqr_controller.R
    )


def set_B(slider):
    global lqr_controller
    lqr_controller.B = slider.value
    lqr_controller.K = lqr_controller.calculate_gain(
        lqr_controller.A, lqr_controller.B, lqr_controller.Q, lqr_controller.R
    )


def set_Q(slider):
    global lqr_controller
    lqr_controller.Q = slider.value
    lqr_controller.K = lqr_controller.calculate_gain(
        lqr_controller.A, lqr_controller.B, lqr_controller.Q, lqr_controller.R
    )


def set_R(slider):
    global lqr_controller
    lqr_controller.R = slider.value
    lqr_controller.K = lqr_controller.calculate_gain(
        lqr_controller.A, lqr_controller.B, lqr_controller.Q, lqr_controller.R
    )


wtext(text="\n")

# Initialize sliders and labels for LQR parameters
wtext(text=" A")
A_slider = slider(min=0.1, max=2, value=1, length=300, bind=set_A)
wtext(text=" B")
B_slider = slider(min=0.1, max=2, value=1, length=300, bind=set_B)
wtext(text=" Q")
Q_slider = slider(min=0.1, max=2, value=1, length=300, bind=set_Q)
wtext(text=" R")
R_slider = slider(min=dt, max=0.5, value=0.1, length=300, bind=set_R)


# Callback functions for the sliders
def set_amplitude(slider):
    global function_generator
    function_generator.set_params(amplitude=slider.value)


def set_frequency(slider):
    global function_generator
    function_generator.set_params(frequency=slider.value)


def set_offset(slider):
    global function_generator
    function_generator.set_params(offset=slider.value)


# Add sliders for target function
wtext(text="\n")
wtext(text=" Amplitude")
amplitude_slider = slider(min=0, max=10, value=5.0, length=300, bind=set_amplitude)
wtext(text=" Frequency")
frequency_slider = slider(min=0, max=10, value=0.1, length=300, bind=set_frequency)
wtext(text=" Offset")
offset_slider = slider(min=0, max=10, value=5.0, length=300, bind=set_offset)


def set_sine():
    global function_generator
    function_generator.set_params(func_type=FunctionType.SINE)


def set_square():
    global function_generator
    function_generator.set_params(func_type=FunctionType.SQUARE)


def set_sawtooth():
    global function_generator
    function_generator.set_params(func_type=FunctionType.SAWTOOTH)


wtext(text="\n")
radio(bind=set_sine, text="Sine", checked=True)
radio(bind=set_square, text="Square")
radio(bind=set_sawtooth, text="Sawtooth")
wtext(text="\n")


# Set Noise Level
def set_noise_level(slider):
    global function_generator
    function_generator.set_params(noise_level=slider.value)


wtext(text=" Noise Level")
noise_level_slider = slider(min=0, max=1, value=0.2, length=300, bind=set_noise_level)

# Initialize flags for controllers
target_enabled = True
pid_enabled = True
bang_enabled = True
p_only_enabled = True
fuzzy_enabled = True
lqr_enabled = True


# Callback functions for checkboxes
def toggle_target(checkbox):
    global target_enabled
    target_enabled = checkbox.checked


def toggle_pid(checkbox):
    global pid_enabled
    pid_enabled = checkbox.checked


def toggle_bang(checkbox):
    global bang_enabled
    bang_enabled = checkbox.checked


def toggle_p_only(checkbox):
    global p_only_enabled
    p_only_enabled = checkbox.checked


def toggle_fuzzy(checkbox):
    global fuzzy_enabled
    fuzzy_enabled = checkbox.checked


def toggle_lqr(checkbox):
    global lqr_enabled
    lqr_enabled = checkbox.checked


wtext(text="\n")

# Initialize checkboxes for enabling/disabling controllers
target_checkbox = checkbox(bind=toggle_target, text="Enable Target", checked=True)
pid_checkbox = checkbox(bind=toggle_pid, text="Enable PID", checked=True)
bang_checkbox = checkbox(bind=toggle_bang, text="Enable Bang-Bang", checked=True)
p_only_checkbox = checkbox(bind=toggle_p_only, text="Enable P-Only", checked=True)
fuzzy_checkbox = checkbox(bind=toggle_fuzzy, text="Enable Fuzzy Logic", checked=True)
lqr_checkbox = checkbox(bind=toggle_lqr, text="Enable LQR", checked=True)

pid_errors = []
bang_errors = []
p_only_errors = []
fuzzy_errors = []
lqr_errors = []

# Create visualizer objects
visualizers = [
    ControlVisualizer(
        pid_error_graph,
        pid_error_graph2,
        pid_error_graph3,
        pid_error_graph4,
        pid_ball,
        pid_controller,
        pid_checkbox,
    ),
    ControlVisualizer(
        bang_error_graph,
        bang_error_graph2,
        bang_error_graph3,
        bang_error_graph4,
        bang_ball,
        bang_controller,
        bang_checkbox,
    ),
    ControlVisualizer(
        p_error_graph,
        p_error_graph2,
        p_error_graph3,
        pid_error_graph4,
        p_only_ball,
        p_only_controller,
        p_only_checkbox,
    ),
    ControlVisualizer(
        fuzzy_error_graph,
        fuzzy_error_graph2,
        fuzzy_error_graph3,
        fuzzy_error_graph4,
        fuzzy_ball,
        fuzzy_controller,
        fuzzy_checkbox,
    ),
    ControlVisualizer(
        lqr_error_graph,
        lqr_error_graph2,
        fuzzy_error_graph3,
        fuzzy_error_graph4,
        lqr_ball,
        lqr_controller,
        lqr_checkbox,
    ),
]


# Main loop
t = 0
while True:
    rate(100)

    target_height = target_function(t)

    # Update controllers and ball positions
    for vis in visualizers:
        if checkbox.checked:
            vis.update_ball(target_height, t)
        else:
            vis.hide_ball()

    # Update target position and graph
    target.pos.y = target_height
    target.pos.z += dt
    if target_enabled:
        target_graph.plot(t, target_height - function_generator.offset, fast=True)

    # Update camera
    scene.camera.pos = vector(scene.camera.pos.x, scene.camera.pos.y, target.pos.z + 10)

    t += dt
