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
        """
        Computes the PID output based on the current feedback value.
        """
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

        # Avoid division by zero
        if dt <= 0:
            dt = 1e-6

        error = self.setpoint - feedback_value

        # 1. Proportional term
        p_term = self.kp * error

        # 2. Integral term (sum of errors over time) - only if there is a small change like 5 degrees
        max_diference_for_integral = 5  # prevents from incrementing when we have sudden changes in target
        if abs(error) < max_diference_for_integral:
            self._integral += error * dt
            self._integral = max(min(self._integral, self.ki_limit), -self.ki_limit)
        else:
            # slowly clean integral when a large change happens
            self._integral *= 0.5

        self._integral += error * dt
        i_term = self.ki * self._integral
        i_term = min(max(i_term, -self.ki_limit), self.ki_limit)

        # 3. Derivative term (rate of change of error)
        derivative = (error - self._prev_error) / dt
        d_term = self.kd * derivative

        # Calculate total output
        output = p_term + i_term + d_term

        # Apply output limits (Saturation)
        if self._max_output is not None:
            output = min(output, self._max_output)
        if self._min_output is not None:
            output = max(output, self._min_output)

        # Save state for next iteration
        self._prev_error = error
        self._last_time = current_time

        return output


# --- Practical Usage Example ---

# Let's say we want to control an RC Elevator (Servo)
# Center position: 90 degrees | Limits: 45 to 135 degrees

#elevator_pid = PID(kp=1.5, ki=0.2, kd=0.05, setpoint=0, output_limits=(-45, 45))

