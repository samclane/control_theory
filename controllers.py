from params import dt

from abc import ABC, abstractmethod


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
            self.dp[i] *= 1.1 * dt
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
                self.dp[i] *= 0.9 * dt


class BangBangController(Controller):
    """Bang-Bang Controller."""

    def __init__(self, delta: float):
        self.delta = delta

    def update(self, error: float) -> float:
        return self.delta if error > 0 else -self.delta

    def tune(self, error: float) -> None:
        # Update delta based on error magnitude
        self.delta += dt * error


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
        # Update delta based on error magnitude
        self.delta += dt * error


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
        self.R += dt/10 * error
        self.K = self.calculate_gain(self.A, self.B, self.Q, self.R)
