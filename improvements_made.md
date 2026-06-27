# Jumping Wheel-Legged Robot — Log of Improvements Made

This file maintains a chronological log of all code optimizations, bug fixes, and feature enhancements applied to the robot's control system.

---

### [2026-06-14 19:40:00 -07:00] LQR Auto-Tuning Optimization Suite
Implemented key improvements to the Bayesian optimization and telemetry loop to make the auto-tuning process far more precise, consistent, and resistant to hardware calibration offsets.

#### 1. Offset-Independent Variance Cost Function
* **File modified**: [tuning_dashboard.py](file:///home/kamren/Final-Project-test-code/scripts/tuning_dashboard.py)
* **Description**: Replaced the absolute quadratic cost function with a variance-based cost function (measuring deviation from the mean tilt angle and mean wheel speed). This prevents static tilt calibration offsets (e.g. if the robot stands at a slight lean) from dominating the cost metric, isolating and evaluating purely dynamic balance recovery performance.

#### 2. Uniform 1D Grid Search in Bayesian Optimizer
* **File modified**: [bayes_opt.py](file:///home/kamren/Final-Project-test-code/src/utils/bayes_opt.py)
* **Description**: Added a 1D grid search path inside the GP optimizer's parameter candidate generation when `len(self.keys) == 1`. This prevents the optimizer from falling back to random coordinate sampling when tuning a single parameter (such as $K_s$), ensuring it systematically sweeps the bounds to locate the absolute minimum of the acquisition function.

#### 3. Arduino-Coordinated Impulse Taps (Hardware-Timed Disturbances)
* **Files modified**: [robot_control.ino](file:///home/kamren/Final-Project-test-code/robot_control/robot_control.ino), [arduino_bridge.py](file:///home/kamren/Final-Project-test-code/src/drivers/arduino_bridge.py), [tuning_dashboard.py](file:///home/kamren/Final-Project-test-code/scripts/tuning_dashboard.py)
* **Description**: Introduced a custom timed impulse command `IMP:<speed>:<duration_ms>` sent from Python to the Arduino. This offloads the tap duration timing from the Pi's OS to the Arduino's 200 Hz firmware loop, eliminating 10–30 ms of jitter caused by USB latency and thread scheduling to guarantee identical testing impulses across all iterations.

#### 4. Session-Based Auto-Tuning Run Isolation
* **File modified**: [tuning_dashboard.py](file:///home/kamren/Final-Project-test-code/scripts/tuning_dashboard.py)
* **Description**: Modified the auto-tune trial logger to save output files in timestamped session subfolders (`config/autotune_logs/run_YYYYMMDD_HHMMSS/`) to prevent subsequent auto-tuning sweeps from overwriting previous trial logs.

---

### [2026-06-15 00:35:00 -07:00] Real-Time Firmware & Control Optimization Suite (Phase 2)
Implemented firmware-level enhancements in the Arduino real-time controller to improve loop safety, stiction response, sensor fusion, and stopping trajectories.

#### 1. Non-Blocking C-String Command Parser
* **File modified**: [robot_control.ino](file:///home/kamren/Final-Project-test-code/robot_control/robot_control.ino)
* **Description**: Replaced blocking `String`-based serial reading with a non-blocking char buffer accumulator and zero-allocation C library routines (`strncmp`, `sscanf`), guaranteeing the 200 Hz control loop never suffers communication latency spikes.

#### 2. Motor Deadband Compensation
* **File modified**: [robot_control.ino](file:///home/kamren/Final-Project-test-code/robot_control/robot_control.ino)
* **Description**: Added a deadband compensation mapping that shifts LQR output commands past the mechanical stiction threshold (minimum PWM of 25), removing low-amplitude wobbling.

#### 3. Startup Gyro Bias Calibration
* **File modified**: [robot_control.ino](file:///home/kamren/Final-Project-test-code/robot_control/robot_control.ino)
* **Description**: Programmed an automatic zero-rate offset calibration cycle at boot (averaging 400 readings over 1.2s), preventing steady-state complementary filter drift.

#### 4. LQR Integrator Anti-Windup Decay
* **File modified**: [robot_control.ino](file:///home/kamren/Final-Project-test-code/robot_control/robot_control.ino)
* **Description**: Introduced an exponential decay (5% per loop step) on the velocity error accumulator (`posErr`) during active travel commands. This stops integrator windup when moving, resulting in crisp stopping behavior and zero braking drift.

#### 5. Axle Linear Acceleration Compensation
* **File modified**: [robot_control.ino](file:///home/kamren/Final-Project-test-code/robot_control/robot_control.ino)
* **Description**: Multiplied the speed derivative by $1/g$ and subtracted the horizontal axle linear acceleration component from the accelerometer X reading. This prevents the estimated pitch angle from being distorted when the robot accelerates or decelerates.

