files = dir('../sensor_data*.csv');

% Initialize an array to store the datetime values
timestamps = NaT(length(files), 1);  % NaT = Not-a-Time, default value for datetime array

% Loop through the files and extract date and time
for i = 1:length(files)
    % Extract date and time portion from the filename (assumes the format is correct)
    tokens = regexp(files(i).name, 'sensor_data_(\d{4}-\d{2}-\d{2})_(\d{2}-\d{2}-\d{2})', 'tokens');
    
    if ~isempty(tokens)
        % Convert to datetime, replacing hyphens with colons for the time part
        dateStr = tokens{1}{1};  % 'YYYY-MM-DD'
        timeStr = strrep(tokens{1}{2}, '-', ':');  % 'HH:MM:SS'
        timestamps(i) = datetime([dateStr ' ' timeStr], 'InputFormat', 'yyyy-MM-dd HH:mm:ss');
    end
end

% Find the most recent timestamp
[~, mostRecentIdx] = max(timestamps);

% Get the filename of the most recent file
mostRecentFile = files(mostRecentIdx).name;

% Display the most recent file
disp(['Most recent file: ', mostRecentFile]);

% Full path to the most recent file
mostRecentFilePath = fullfile(files(mostRecentIdx).folder, mostRecentFile);

% Load the CSV file
data = readtable(mostRecentFilePath, 'VariableNamingRule', 'preserve');

time = data.("Time (s)");
accelX = data.("Accel X");
accelY = data.("Accel Y");
accelZ = data.("Accel Z");
gyroX = data.("Gyro X");
gyroY = data.("Gyro Y");
gyroZ = data.("Gyro Z");
temperature = data.("Temperature (C)");

% Compute the sample intervals (time differences between consecutive samples)
timeDiffs = diff(time);  % Time differences between consecutive samples

% Calculate sample frequency
sampleFrequency = 1 ./ mean(timeDiffs);  % Inverse of the mean time difference

% Calculate precision metrics (standard deviation of time intervals)
sampleStdDev = std(timeDiffs);

% Display metrics in the terminal
fprintf('Sample frequency: %.2f Hz\n', sampleFrequency);
fprintf('Average time interval: %.6f seconds\n', mean(timeDiffs));
fprintf('Time interval standard deviation: %.6f seconds\n', sampleStdDev);

% Compute the integrals of accelZ
velocityZ = cumtrapz(time, accelZ);  % Numerical integration using cumulative trapezoidal method
distZ = cumtrapz(time, velocityZ);  

% Plot Acceleration Z over Time (First Plot)
figure;  % Create a new figure window
plot(time, accelZ);
xlabel('Time (s)');
ylabel('Accel Z (m/s^2)');
title('Acceleration Z over Time');
xlim([0 2]);        % Set time limit from 0 to 2 seconds
ylim([-12 12]);     % Set acceleration limit from -12 to 12 m/s^2

% Plot Velocity Z over Time (Second Plot - integral of Accel Z)
figure;  % Create another figure window
plot(time, velocityZ);
xlabel('Time (s)');
ylabel('Velocity Z (m/s)');
title('Velocity Z over Time');
xlim([0 2]);        % Set time limit from 0 to 2 seconds
ylim([0 1.8]);      % Set velocity limit from 0 to 1.8 m/s
