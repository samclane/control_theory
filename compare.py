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
)
from abc import ABC, abstractmethod

dt = 0.01  # Time step


# Abstract Controller class
class Controller(ABC):
    """Abstract Base Class for controllers."""

    @abstractmethod
    def update(self, error: float) -> float:
        """Update the control input based on the error."""
        pass

    @abstractmethod
    def tune(self, error: float) -> None:
        """Tune the controller parameters based on the error."""
        pass


# Control Algorithm Classes
class PIDController(Controller):
    """Proportional-Integral-Derivative Controller."""

    def __init__(self, Kp: float, Ki: float, Kd: float):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0
        self.integral = 0
        self.dp = [0.1, 0.1, 0.1]  # For twiddle algorithm

    def update(self, error: float) -> float:
        self.integral += error
        derivative = error - self.prev_error
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.prev_error = error
        return output

    def tune(self, error: float) -> None:
        best_err = error
        for i in range(3):
            self.dp[i] *= 1.1
            if i == 0:
                self.Kp += self.dp[i]
            elif i == 1:
                self.Ki += self.dp[i]
            else:
                self.Kd += self.dp[i]

            err = error  # Replace with the actual error after running the system

            if err < best_err:
                best_err = err
            else:
                self.dp[i] *= 0.9


class BangBangController(Controller):
    """Bang-Bang Controller."""

    def __init__(self, delta: float):
        self.delta = delta

    def update(self, error: float) -> float:
        return self.delta if error > 0 else -self.delta

    def tune(self, error: float) -> None:
        # Update delta based on error magnitude
        self.delta += 0.1 * error


class POnlyController(Controller):
    """Proportional-Only Controller."""

    def __init__(self, Kp: float):
        self.Kp = Kp

    def update(self, error: float) -> float:
        return self.Kp * error

    def tune(self, error: float) -> None:
        # Update Kp based on error magnitude
        self.Kp += dt * error


class FuzzyLogicController(Controller):
    """Fuzzy Logic Controller."""

    def __init__(self, delta: float):
        self.delta = delta
        self.prev_error = 0
        self.min_delta = 0.1
        self.max_delta = 10.0

    def calculate_update(self, error):
        if error > 2:
            return 0.1
        elif error > 1:
            return 0.05
        elif error > 0:
            return 0.02
        elif error > -1:
            return -0.02
        elif error > -2:
            return -0.05
        else:
            return -0.1

    def update(self, error):
        return self.calculate_update(error) * self.delta

    def tune(self, error: float) -> None:
        pass


class LQRController(Controller):
    """Linear Quadratic Regulator (LQR) Controller."""

    def __init__(self, A: float, B: float, Q: float, R: float):
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R
        self.K = self.calculate_gain(A, B, Q, R)

    def calculate_gain(self, A: float, B: float, Q: float, R: float) -> float:
        """Calculate the LQR gain."""
        return (B**2 * Q + R) / (B * (A**2 * Q + Q))

    def update(self, state: float) -> float:
        control_input = self.K * state
        return control_input

    def tune(self, error: float) -> None:
        # Update Q and R based on error and recalculate K
        self.Q += dt * error
        self.R += 0.001 * error
        self.K = self.calculate_gain(self.A, self.B, self.Q, self.R)


def update_ball(
    ball: sphere, controller: Controller, target_height: float, t: float, graph: gcurve
) -> None:
    """Update the position of the ball based on the controller."""
    error = target_height - ball.pos.y
    output = controller.update(error)
    controller.tune(error)
    ball.pos.y += output * dt
    graph.plot(t, error)


def hide_ball(ball: sphere) -> None:
    """Hide the ball by setting its opacity to 0."""
    ball.opacity = 0


def target_function(t: float) -> float:
    """Calculate the target height based on time."""
    return offset + amplitude * sin(frequency * t)


# Initialize graphs
target_graph = gcurve(color=color.yellow, label="Target Height")
pid_error_graph = gcurve(color=color.red, label="PID Error")
bang_error_graph = gcurve(color=color.green, label="Bang-Bang Error")
p_error_graph = gcurve(color=color.blue, label="P-Only Error")
fuzzy_error_graph = gcurve(color=color.purple, label="Fuzzy Logic Error")
lqr_error_graph = gcurve(color=color.magenta, label="LQR Error")


# Initialize simulation
amplitude = 5.0
frequency = 0.1
offset = 5.0

pid_ball = sphere(pos=vector(-5, 0, 0), radius=1, color=color.red)
bang_ball = sphere(pos=vector(0, 0, 0), radius=1, color=color.green)
p_only_ball = sphere(pos=vector(5, 0, 0), radius=1, color=color.blue)
fuzzy_ball = sphere(pos=vector(10, 0, 0), radius=1, color=color.purple)
lqr_ball = sphere(pos=vector(15, 0, 0), radius=1, color=color.magenta)


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
def target_slider_changed(slider):
    global target_height
    target_height = slider.value
    target.pos.y = target_height


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
wtext(text=" Target Height")
target_slider = slider(min=0, max=10, value=5, length=300, bind=target_slider_changed)

wtext(text="\n")

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
    global amplitude
    amplitude = slider.value


def set_frequency(slider):
    global frequency
    frequency = slider.value


def set_offset(slider):
    global offset
    offset = slider.value


# Add sliders for target function
wtext(text="\n")
wtext(text=" Amplitude")
amplitude_slider = slider(min=0, max=10, value=5.0, length=300, bind=set_amplitude)
wtext(text=" Frequency")
frequency_slider = slider(min=0, max=1, value=0.1, length=300, bind=set_frequency)
wtext(text=" Offset")
offset_slider = slider(min=0, max=10, value=5.0, length=300, bind=set_offset)


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


controllers_and_balls = [
    (pid_controller, pid_ball, pid_error_graph, pid_checkbox),
    (bang_controller, bang_ball, bang_error_graph, bang_checkbox),
    (p_only_controller, p_only_ball, p_error_graph, p_only_checkbox),
    (fuzzy_controller, fuzzy_ball, fuzzy_error_graph, fuzzy_checkbox),
    (lqr_controller, lqr_ball, lqr_error_graph, lqr_checkbox),
]

# Main loop
t = 0
while True:
    rate(100)

    target_height = target_function(t)

    # Update controllers and ball positions
    for controller, ball, graph, checkbox in controllers_and_balls:
        if checkbox.checked:
            update_ball(ball, controller, target_height, t, graph)
        else:
            hide_ball(ball)

    # Update target position and graph
    target.pos.y = target_height
    if target_enabled:
        target_graph.plot(t, target_height - offset)

    t += dt
