import time

class PID:
    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(None, None), ki_limit=150):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.goal = setpoint
        self.setpoint = setpoint
        self._min_output, self._max_output = output_limits
        self.ki_limit = ki_limit

        self.iteration = 999

        self._integral = 0
        self._prev_error = 0
        self._last_time = time.time()

    def soft_set_target(self, new_target, iteration):
        self.goal = new_target
        self.iteration = iteration

    def update(self, feedback_value):
        """Computes the PID output based on current feedback."""
        current_time = time.time()

        if self.setpoint != self.goal:
            if self.goal > self.setpoint:
                self.setpoint += min(float(self.goal - self.setpoint), self.iteration)
                if feedback_value > self.setpoint:
                    self.setpoint = min(feedback_value, self.goal)

            elif self.goal < self.setpoint:
                self.setpoint -= min(float(self.setpoint - self.goal), self.iteration)
                if feedback_value < self.setpoint:
                    self.setpoint = max(feedback_value, self.goal)

        dt = current_time - self._last_time
        if dt <= 0:
            dt = 1e-6

        error = self.setpoint - feedback_value

        p_term = self.kp * error

        # Integral term (only for small changes)
        max_diference_for_integral = 5  
        if abs(error) < max_diference_for_integral:
            self._integral += error * dt
            self._integral = max(min(self._integral, self.ki_limit), -self.ki_limit)
        else:
            self._integral *= 0.5

        self._integral += error * dt
        i_term = self.ki * self._integral
        i_term = min(max(i_term, -self.ki_limit), self.ki_limit)

        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        if self._max_output is not None:
            output = min(output, self._max_output)
        if self._min_output is not None:
            output = max(output, self._min_output)

        self._prev_error = error
        self._last_time = current_time

        return output


class AdaptivePID:
    """Non-Linear Adaptive PID. Self-adjusts gains in real-time."""

    def __init__(self, kp, ki, kd, setpoint=0, output_limits=(None, None), ki_limit=150):
        self.base_kp = kp
        self.base_ki = ki
        self.base_kd = kd

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.goal = setpoint
        self.setpoint = setpoint
        self._min_output, self._max_output = output_limits
        self.ki_limit = ki_limit

        self.iteration = 999

        self._integral = 0
        self._prev_error = 0
        self._last_time = time.time()

    def soft_set_target(self, new_target, iteration):
        self.goal = new_target
        self.iteration = iteration

    def update(self, feedback_value):
        """Computes Adaptive output."""
        current_time = time.time()

        if self.setpoint != self.goal:
            if self.goal > self.setpoint:
                self.setpoint += min(float(self.goal - self.setpoint), self.iteration)
                if feedback_value > self.setpoint:
                    self.setpoint = min(feedback_value, self.goal)

            elif self.goal < self.setpoint:
                self.setpoint -= min(float(self.setpoint - self.goal), self.iteration)
                if feedback_value < self.setpoint:
                    self.setpoint = max(feedback_value, self.goal)

        dt = current_time - self._last_time
        if dt <= 0:
            dt = 1e-6

        error = self.setpoint - feedback_value
        abs_error = abs(error)

        # Auto-calibration of gains
        self.kp = self.base_kp * (1.0 + 0.1 * abs_error)
        self.ki = self.base_ki / (1.0 + 0.5 * abs_error)
        self.kd = self.base_kd * (1.0 + 0.2 * abs_error)

        p_term = self.kp * error

        self._integral += error * dt
        i_term = self.ki * self._integral
        i_term = min(max(i_term, -self.ki_limit), self.ki_limit)

        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        if self._max_output is not None:
            output = min(output, self._max_output)
        if self._min_output is not None:
            output = max(output, self._min_output)

        self._prev_error = error
        self._last_time = current_time

        return output
