graph TD
Start([Start])-- > Init[Initialize Hardware: I2C, UART, PWM, LED]
Init-- > Controllers[Setup Kalman Filter & PID Controllers]
Controllers-- > Watchdog[Start Watchdog Timer]
Watchdog-- > Loop{Main Loop }

%% Main Path
Loop-- > RC[Read RC Packets via UART]
Loop-- > IMU[Read & Process IMU Data]
%% Failsafe Check
IMU-- > FailsafeCheck{Signal Lost > 500ms ?}
FailsafeCheck-- Yes-- > Failsafe[FAILSAFE: Neutral Servos & Zero Throttle]
FailsafeCheck-- No-- > StateMachine{Current State ?}

%% State Machine
StateMachine-- CALIBRATION-- > Calib[Calculate Gyro Bias & Update LED]
StateMachine-- FLIGHT_MODE-- > ArmedCheck{ Armed ?}
ArmedCheck-- No-- > Disarmed[Lock ESC at Minimum]
ArmedCheck-- Yes-- > ModeCheck{Manual Mode ?}

ModeCheck-- Yes-- > Manual[Direct RC - to - Servo Pass - through]
ModeCheck-- No-- > Stabilized[Stabilized Mode]

%% Stabilization Logic
Stabilized-- > Fusion[Kalman Filter Attitude Fusion]
Fusion-- > PID[Calculate PID Error: Desired Rate - IMU Rate]
PID-- > Mix[Elevon Mixing: Pitch Output +/- Roll Output]
Mix-- > PWM[Apply Subtrims & Constrain PWM Outputs]

%% Feedback Loop
Failsafe-- > WatchdogUpdate[Update Watchdog]
Calib-- > WatchdogUpdate
Disarmed-- > WatchdogUpdate
Manual-- > WatchdogUpdate
PWM-- > WatchdogUpdate
WatchdogUpdate-- > Loop