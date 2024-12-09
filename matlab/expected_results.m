% Parameters
acceleration = -5;    % Negative acceleration in m/s^2 (5 m/s^2 downwards)
max_speed = -0.1;     % Negative constant speed in m/s (0.1 m/s downwards)
deceleration = 5;     % Positive deceleration in m/s^2 (for stopping, same magnitude as acceleration)

% Time duration for acceleration and deceleration phases
t_accel = abs(max_speed / acceleration);  % Time to reach max speed
t_decel = abs(max_speed / deceleration);  % Time to decelerate to zero

% Total time (fixed to 2 seconds)
total_time = 1.42;  % Total time for the entire motion

% Calculate time for constant speed to ensure the total time is exactly 2 seconds
t_const = total_time - 0.4 - t_accel - t_decel;  % Adjust constant speed time

end_time = 2.0

% Time vector (shift start of acceleration to t = 0.4s)
time = linspace(0, end_time, 1000);  % Time points for plotting

% Initialize acceleration and velocity vectors
accelZ = zeros(size(time));
velocityZ = zeros(size(time));

% Loop through the time and calculate acceleration and velocity
for i = 1:length(time)
    if time(i) > 0.4 && time(i) <= 0.4 + t_accel  % Acceleration phase (shifted start)
        accelZ(i) = acceleration;
        velocityZ(i) = acceleration * (time(i) - 0.4);  % Start counting time from t=0.4
    elseif time(i) > 0.4 + t_accel && time(i) <= 0.4 + t_accel + t_const  % Constant speed phase
        accelZ(i) = 0;
        velocityZ(i) = max_speed;
    elseif time(i) > 0.4 + t_accel + t_const && time(i) <= end_time  % Deceleration phase
        accelZ(i) = -deceleration;
        velocityZ(i) = max_speed + deceleration * (time(i) - (0.4 + t_accel + t_const));
    end
end

% Plot Acceleration in Z Direction (negative direction)
figure;
plot(time, accelZ, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Expected Accel Z (m/s^2)');
title('Expected Acceleration Z over Time');
xlim([0 2]);        % Set time limit from 0 to 2 seconds
ylim([-12 12]);     % Set acceleration limit from -12 to 12 m/s^2
grid on;

% Plot Velocity in Z Direction (negative direction)
figure;
plot(time, velocityZ, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Expected Velocity Z (m/s)');
title('Expected Velocity Z over Time');
xlim([0 2]);        % Set time limit from 0 to 2 seconds
ylim([-0.5 0]);     % Set velocity limit from -0.1 to 0 m/s
grid on;
