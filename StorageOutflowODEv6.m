clear
pressurepumped = 3; %Amount of bar the tank was pressurized to
V_0 = 5*10^-4; %[m^3], volume filled
p_0 = 1.5*10^5; %[Pa], starting pressure in tank filled with air


data = {
    '14:34:06.518 -> 	0L Pressure (Bar)2.95Flow rate: 0L/min	Output Liquid Quantity: 0mL'
'14:34:07.522 -> 	0L Pressure (Bar)3.00Flow rate: 0L/min	Output Liquid Quantity: 0mL'
'14:34:08.548 -> 	0L Pressure (Bar)3.00Flow rate: 0L/min	Output Liquid Quantity: 0mL'
'14:34:09.525 -> 	0L Pressure (Bar)3.00Flow rate: 0L/min	Output Liquid Quantity: 0mL'
'14:34:10.520 -> 	0L Pressure (Bar)3.00Flow rate: 0L/min	Output Liquid Quantity: 1mL'
'14:34:11.528 -> 	0L Pressure (Bar)2.95Flow rate: 1L/min	Output Liquid Quantity: 29mL'
'14:34:12.540 -> 	0L Pressure (Bar)2.37Flow rate: 0L/min	Output Liquid Quantity: 32mL'
'14:34:13.527 -> 	0L Pressure (Bar)2.05Flow rate: 7L/min	Output Liquid Quantity: 155mL'
'14:34:14.539 -> 	0L Pressure (Bar)1.84Flow rate: 6L/min	Output Liquid Quantity: 266mL'
'14:34:15.530 -> 	0L Pressure (Bar)1.74Flow rate: 6L/min	Output Liquid Quantity: 369mL'
'14:34:16.542 -> 	0L Pressure (Bar)1.63Flow rate: 0L/min	Output Liquid Quantity: 378mL'
'14:34:17.562 -> 	0L Pressure (Bar)1.32Flow rate: 3L/min	Output Liquid Quantity: 435mL'
'14:34:18.570 -> 	0L Pressure (Bar)1.05Flow rate: 7L/min	Output Liquid Quantity: 560mL'
'14:34:19.557 -> 	0L Pressure (Bar)1.00Flow rate: 0L/min	Output Liquid Quantity: 562mL'
'14:34:20.552 -> 	0L Pressure (Bar)1.00Flow rate: 0L/min	Output Liquid Quantity: 562mL'
'14:34:21.539 -> 	0L Pressure (Bar)1.00Flow rate: 0L/min	Output Liquid Quantity: 562mL'
'14:34:22.578 -> 	0L Pressure (Bar)1.00Flow rate: 0L/min	Output Liquid Quantity: 562mL'
'14:34:23.555 -> 	0L Pressure (Bar)1.00Flow rate: 0L/min	Output Liquid Quantity: 562mL'
};

% Initialize empty column vectors
testtime_vector = zeros(numel(data), 1);
testpressure_vector = zeros(numel(data), 1);
testcumvolume_vector = zeros(numel(data), 1);

% Initialize a flag to check if the time should start counting
start_counting = false;

% Extract the initial time as a datenum
initial_time = datenum(data{1}(1:12), 'HH:MM:SS.FFF');

% Split and extract data
for i = 1:numel(data)
    parts = strsplit(data{i}, ' -> ');
    time_str = parts{1};
    data_str = parts{2};

    % Extract pressure and output liquid quantity using regular expressions
    pressure_match = regexp(data_str, 'Pressure \(Bar\)([0-9.]+)', 'tokens');
    quantity_match = regexp(data_str, 'Output Liquid Quantity: ([0-9]+)mL', 'tokens');
    
    current_time = datenum(time_str, 'HH:MM:SS.FFF');
    
    % Calculate the time difference from the initial time
    if ~start_counting
        if i > 5
            start_counting = true;  % Start counting when volumeflow is larger than 0
            initial_time = current_time;  % Update initial time
        else
            continue;  % Skip data points with volumeflow <= 0
        end
    end
    
    testtime_vector(i) = (current_time - initial_time) * 86400;  % Convert to seconds to start at 0
    
    if ~isempty(pressure_match)
        testpressure_vector(i) = str2double(pressure_match{1}{1});
    else
        testpressure_vector(i) = 0.0;  % Handle missing data
    end
    
    if ~isempty(quantity_match)
        testcumvolume_vector(i) = str2double(quantity_match{1}{1});
    else
        testcumvolume_vector(i) = 0;  % Handle missing data
    end
end

testcumvolume_m3 = testcumvolume_vector * 10^-6;   % Calculates cumulative outflown volume in m^3 instead of ml
testpressure_pascal = (1) * testpressure_vector * 10^5; % Calculates pressure in Pascal
testpressure_pascal(testpressure_pascal < p_0) = p_0;

testvolumeflow_m3 = diff(testcumvolume_m3); % Calculate the difference to get incremental volume
testvolumeflow_m3 = [0; testvolumeflow_m3]; % Add a zero at the beginning to match the size



% Initialize an array to store the tank volume over time
tank_volume_m3 = zeros(size(testtime_vector));

% Initialize the tank volume with the initial volume
tank_volume_m3(1) = V_0;


% Integrate the negative of the volumeflow to find the tank volume over time
for i = 2:numel(testtime_vector)
    delta_t = testtime_vector(i) - testtime_vector(i - 1);
    tank_volume_m3(i) = tank_volume_m3(i - 1) - testvolumeflow_m3(i) * delta_t;
end

tank_volume_m3(tank_volume_m3 < 0) = 0;
tank_volume_m3 = tank_volume_m3(2:end);
tank_volume_m3 = [tank_volume_m3; 0];


% Create a new figure to plot the tank volume over time
figure;
plot(testtime_vector, tank_volume_m3);
xlabel('Time (s)');
ylabel('Tank Volume (m^3)');
title('Tank Volume Over Time');
grid on;




% Create a new figure for testvolumeflow_m3
figure;

% Create a subplot with 2 rows and 1 column, select the first subplot
subplot(2, 1, 1);
plot(testtime_vector, testvolumeflow_m3);
xlabel('Time (s)');
ylabel('Tested Volumeflow (m^3/s)');
title('Tested Volumeflow Over Time');
grid on;

% Create a subplot with 2 rows and 1 column, select the second subplot
subplot(2, 1, 2);
plot(testtime_vector, testpressure_pascal);
xlabel('Time (s)');
ylabel('Tested Pressure (Pa)');
title('Tested Pressure Over Time');
grid on;






% Define the parameters
A_1 = 1.63*10^-2; %[m^2], cross-sectional area of the tank
A_2 = 1/8*5.03*10^-5; %[m^2], cross-sectional area of the outflow pipe
V_tank = 1*10^-3; %[m^3], total volume of the tank
%p_0 = 1.5*10^5; %[Pa], starting pressure in tank filled with air
p_2 = 1*10^5; %[Pa], pressure at the outflow pipe

%pressurepumped*(1-(V_0/V_tank))

g = 9.81; %[m/s^2], gravitational constant on earth
rho_water = 1000; %[kg/m^3], density of water

% Define the ODE function
ode_func = @(t, y) -sqrt( 1/((A_1^2/A_2^2)-1)) * sqrt((2 * p_0 / (rho_water * (1 - A_1 * y / V_tank))) + (2 * g * y) - (2 * p_2 / rho_water));


% Define the time span
tspan = [0, 10]; % Adjust the time span as needed

% Initial conditions of water level
y0 = V_0/A_1; % h_1(0)

% Define options to enforce non-negativity
options = odeset('NonNegative', 1); % Enforce non-negativity for y(1)

% Solve the ODE with the NonNegative option
[t, h_1] = ode45(ode_func, tspan, y0, options);

% Define a function for V & p
V_function = @(h_1) A_1 * h_1; % Calculates water volume
p_function = @(h_1) p_0 ./ (1 - (V_function(h_1) ./ V_tank)); % Calculates air pressure in the vessel

% Calculate p_function values
V_function_values = V_function(h_1);
p_function_values = p_function(h_1);

% Initialize arrays to store results
indexlength = length(t);
Volumeflow = zeros(indexlength - 1, 1);
deltaV_array = zeros(indexlength - 1, 1);
t_derivative = zeros(indexlength - 1, 1);

% Loop through the time points
for i = 2:indexlength
    dt = t(i) - t(i-1);
    deltaV = V_function_values(i) - V_function_values(i-1);
    
    % Calculate the derivative and store the results
    Volumeflow(i-1) = -(deltaV / dt);
    deltaV_array(i-1) = deltaV;
    t_derivative(i-1) = t(i-1); % Store the corresponding time point
end


% Plot the results
figure;

% Plot h_1
subplot(4, 1, 1);
plot(t, h_1);
xlabel('Time (t)');
ylabel('Water level [m]');
title('Water level vs. Time');

% Plot V_function_values
subplot(4, 1, 2);
plot(t, V_function_values);
xlabel('Time (t)');
ylabel('Present water [m^3]');
title('Present water vs. Time');

% Plot p_function_values
subplot(4, 1, 3);
plot(t, p_function_values);
xlabel('Time (t)');
ylabel('Pressure (Pa)');
title('Pressure vs. Time');

% Plot Volumeflow_out 
subplot(4, 1, 4);
plot(t_derivative, Volumeflow);
xlabel('Time (t)');
ylabel('Volume Flow Out [m^3/s]');
title('Volume Flow Out vs. Time');

% Adjust plot appearance
grid on;





% Create a new figure for the superimposed plot
figure;

% Plot the measured volumeflow_m3
plot(testtime_vector, tank_volume_m3, 'b', 'DisplayName', 'Measured Volume');
hold on;

% Plot the modeled volumeflow using h_1 and t_derivative
plot(t, V_function_values, 'r', 'DisplayName', 'Model Volume');
hold off;

xlim([0, 10]);

xlabel('Time (s)');
ylabel('Volume (m^3)');
title('Measured vs. Modeled Present Volume');
legend;
grid on;





% Create a new figure for the superimposed plot
figure;

% Plot the measured pressure
plot(testtime_vector, testpressure_pascal, 'b', 'DisplayName', 'Measured Pressure');
hold on;

% Plot the modeled pressure
plot(t, p_function_values, 'r', 'DisplayName', 'Model Pressure');
hold off;

xlim([0, 10]);

xlabel('Time (s)');
ylabel('Pressure (Pascal)');
title('Measured vs. Modeled Pressure');
legend;
grid on;

