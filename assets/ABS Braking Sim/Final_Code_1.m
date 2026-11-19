%% Final Simulation of ABS System - ME 3017-D - Spring 2025
% Description: The script models a 4-wheeled car's nonlinear and linearized ABS system with three
% different brake inputs (angles) and one coefficent of friction input. The output
% of the script creates six plots of vehicle's acceleration as a result of the
% three different methods and respective nonlinear and linearized model 
% of braking and reaction of the ABS. Also, a subplot plotting the angle
% inputs is created.
%
% Created by: Josh Chung, KC Hinz, and Stephen Jones, 4/12/2025


% Clear workspace and console
clear; clc

% Initialize variables
g = 9.81; % acceleration due to gravity (m/s^2)
m = 1300; % mass of car (kg)
mu_pr = 0.4; % pad and rotor coeffiecent of friction
d_m = 0.05; % diameter of piston in master cylinders (m)
d_s = 2 * d_m; % diameter of pistons in caliper--slave cylinder (m)
pedal_r = 0.7; % inverse pedal ratio
r_ro = 0.165; % brake disc outer radius (m)
r_ri = 0.115; % brake disc inner radius (m)
r_t = 0.65; % radius of the tire (m)
% F_p = 500; % force from driver on to the pedal (N)
p_a = pi/12; % 15 deg; pedal angle caused by driver's foot on the pedal (rad)
fr_r_u = 0.9; % upper coefficent of friction of the road
fr_r_l = 0.8; % lower coefficent of friction of the road
fr_i_u = 0.4; % upper coefficent of friction of ice
fr_i_l = 0.2; % lower coefficent of friction of ice
K = 1 * 10^6; % bulk modulus (Pa)
l_1 = 0.075; % top length of the pedal assembly (m)
l_p = 0.075; % length of the brake cylinder (m)

% Define time vector
t = 0:.1:60; % time from 0 to 60 seconds

% Define friction change input
step_ice = zeros(size(t));
step_ice(t <= 30) = fr_r_l + (fr_r_u - fr_r_l) .* rand(1,length(t(t <= 30)));
step_ice(t > 30) = fr_i_l + (fr_i_u - fr_i_l) .* rand(1,length(t(t > 30)));

% Friction input without randomness
%step_ice(t <= 30) = .9;
%step_ice(t > 30) = .4;

%% Create Braking Input Functions

% Ramp input function
ramp = zeros(size(t));
ramp(10 <= t & (t < 20)) = (p_a/10) * t(t < 10); % Linear increase to lower F_p in 2 seconds
ramp(t >= 20) = p_a; % Hold at lower F_p for the rest of the time

% Step input function
step = zeros(size(t));
step(t >= 10) = p_a; % Step up to F_p at t = 2s

% Sinusoidal input function
cos_wave = zeros(size(t));
cos_wave(t >= 10) = (p_a * (-cos(pi*t(t <= 50)) + 1) / 2);

%% Plot Input Functions
figure;

% Friction change function subplot
subplot(4,1,1);
plot(t, step_ice, 'm', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Coefficent of Friction'); title('Coefficent of Friction Input Function');
grid on; xlim([0 60]); ylim([0 1]); 

% Ramp function subplot
subplot(4,1,2);
plot(t, ramp, 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)'); title('Ramp Input Function');
grid on; xlim([0 60]); ylim([0 .5]); 

% Step function subplot
subplot(4,1,3);
plot(t, step, 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)'); title('Step Input Function');
grid on; xlim([0 60]); ylim([0 .5]);

% Sinusodal function subplot
subplot(4,1,4);
plot(t, cos_wave, 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Angle (rad)'); title('Sinusoidal Input Function');
grid on; xlim([0 60]); ylim([0 .5]);

%% Nonlinear ABS Simulation

% Define pressure coefficent (needs *input force)
p_c = (K * l_1)/l_p; % pedal assembly pressure coeffiecent

% Braking Force Transfer Function (Gain)
coef1 = (mu_pr*pi*(d_s.^2))/(3*r_t);
coef2 = ((r_ro.^3) - (r_ri.^3)) / ((r_ro.^2) - (r_ri.^2));
F_b_c = (coef1) * (coef2); % essentially braking force coefficent

% Acceration of the 4-Wheeled Vehicle Transfer Function (Gain)
A_v = zeros(size(t));

% Initialize feedback inputs to avoid errors
F_b = 0; 
F_fb = 0; % difference of frictional force and braking force
p_b1 = 0; % p_b retainer if input force is held constant

% Begin running ABS system for ramp input
for ii = 1:1:length(t)
    if ramp(ii) > 0

        % Pedal Assembly block
        p_b = p_c .* sin(ramp(ii));
        if ramp(ii) == ramp(ii-1)
            p_b = p_b1; % retains previous p_b if input force is the same
        end
        
        % Tire block
        F_f = m .* g .* step_ice(ii);
        
        % ABS Module block
        if F_fb > 0.0001
            p_b = 1.10 .* p_b;
        elseif F_fb < -0.0001
            p_b = 0.90 .* p_b;
        end
        
        % Brake Assembly block
        F_b = p_b .* F_b_c;
    
        % 4 Wheels Gain block
        F_b = 4 .* F_b;
        
        % Summer Operator
        F_fb = F_f - F_b;

        p_b1 = p_b; % remember current p_b value
        
    end

    % Vehicle block, Acceleration Output
    A_v(ii) = F_b ./ m;
    
end

% Create plot
figure;
plot(t, A_v, 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Nonlinear Ramp Input Output Function');
grid on;


% Initialize feedback inputs to avoid errors
F_b = 0; 
F_fb = 0; % difference of frictional force and braking force
p_b1 = 0; % p_b retainer if input force is held constant

% Begin running ABS system for step input
for ii = 1:1:length(t)
    if step(ii) > 0

        % Pedal Assembly block
        p_b = p_c .* sin(step(ii));
        if step(ii) == step(ii-1)
            p_b = p_b1; % retains previous p_b if input force is the same
        end

        % Tire block
        F_f = m .* g .* step_ice(ii);
    
        % ABS Module block
        if F_fb > 0.0001
            p_b = 1.1 .* p_b;
        elseif F_fb < -0.0001
            p_b = 0.9 .* p_b;
        end
    
        % Brake Assembly block
        F_b = p_b .* F_b_c;
    
        % 4 Wheels Gain block
        F_b = 4 .* F_b;
    
        % Summer Operator
        F_fb = F_f - F_b;

        p_b1 = p_b; % remember current p_b value
    end

    % Vehicle block, Acceleration Output
    A_v(ii) = F_b ./ m;

end

% Create plot
figure;
plot(t, A_v, 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Nonlinear Step Input Output Function');
grid on;

% Initialize feedback inputs to avoid errors
F_b = 0; 
F_fb = 0; % difference of frictional force and braking force
p_b = 0; % initial brake pressure
p_b1 = 0; % p_b retainer if input force is held constant

% Begin running ABS system for sinusdal input
for ii = 1:length(t)

    % Get driver's input from pedal
    if cos_wave(ii) > 0
        p_in = p_c * sin(cos_wave(ii));
    else
        p_in = 0;
    end

    % Tire block
    F_f = m * g * step_ice(ii);

    % Brake assembly block
    F_b = p_b * F_b_c;
           
    % 4 Wheels Gain block
    F_b = 4 * F_b;

    % Summer Operator
    F_fb = F_f - F_b;

    % Define gain and pressure change
    kp = .5; % proportional gain
    dp = kp * F_fb; % change in pressure

    % ABS Module block
    p_b = p_b + dp;
    p_b = min(max(p_b, 0), p_in); % ABS can not increase more than driver input
    
    % Save for next iteration
    p_b1 = p_b;

    % Vehicle block, Acceleration Output
    A_v(ii) = 20 * F_b / m; % includes gain of 20

end


% Create plot
figure;
plot(t, A_v, 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Nonlinear Sinusodal Input Output Function');
grid on;


%% Linearized ABS Simulation

% Acceration of the 4-Wheeled Vehicle Transfer Function (Gain)
A_v = zeros(size(t));

% Initialize feedback inputs to avoid errors
F_b = 0; 
F_fb = 0; % difference of frictional force and braking force
p_b1 = 0; % p_b retainer if input force is held constant

% Begin running ABS system for ramp input
for ii = 1:1:length(t)
    if ramp(ii) > 0

        % Pedal Assembly block
        p_b = p_c .* ramp(ii);
        if ramp(ii) == ramp(ii-1)
            p_b = p_b1; % retains previous p_b if input force is the same
        end
        
        % Tire block
        F_f = m .* g .* step_ice(ii);
        
        % ABS Module block
        if F_fb > 0.0001
            p_b = 1.10 .* p_b;
        elseif F_fb < -0.0001
            p_b = 0.90 .* p_b;
        end
        
        % Brake Assembly block
        F_b = p_b .* F_b_c;
    
        % 4 Wheels Gain block
        F_b = 4 .* F_b;
        
        % Summer Operator
        F_fb = F_f - F_b;

        p_b1 = p_b; % remember current p_b value
        
    end

    % Vehicle block, Acceleration Output
    A_v(ii) = F_b ./ m;
    
end

% Create plot
figure;
plot(t, A_v, 'r', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Linearized Ramp Input Output Function');
grid on;


% Initialize feedback inputs to avoid errors
F_b = 0; 
F_fb = 0; % difference of frictional force and braking force
p_b1 = 0; % p_b retainer if input force is held constant

% Begin running ABS system for step input
for ii = 1:1:length(t)
    if step(ii) > 0

        % Pedal Assembly block
        p_b = p_c .* step(ii);
        if step(ii) == step(ii-1)
            p_b = p_b1; % retains previous p_b if input force is the same
        end

        % Tire block
        F_f = m .* g .* step_ice(ii);
    
        % ABS Module block
        if F_fb > 0.0001
            p_b = 1.10 .* p_b;
        elseif F_fb < -0.0001
            p_b = 0.90 .* p_b;
        end
    
        % Brake Assembly block
        F_b = p_b .* F_b_c;
    
        % 4 Wheels Gain block
        F_b = 4 .* F_b;
    
        % Summer Operator
        F_fb = F_f - F_b;

        p_b1 = p_b; % remember current p_b value
    end

    % Vehicle block, Acceleration Output
    A_v(ii) = F_b ./ m;

end

% Create plot
figure;
plot(t, A_v, 'g', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Linearized Step Input Output Function');
grid on;

% Initialize feedback inputs to avoid errors
F_b = 0; 
F_fb = 0; % difference of frictional force and braking force
p_b = 0; % initial brake pressure
p_b1 = 0; % p_b retainer if input force is held constant

% Begin running ABS system for sinusodal input
for ii = 1:length(t)

    % Get driver's input from pedal
    if cos_wave(ii) > 0
        p_in = p_c * cos_wave(ii);
    else
        p_in = 0;
    end

    % Tire block
    F_f = m * g * step_ice(ii);

    % Brake assembly block
    F_b = p_b * F_b_c;
           
    % 4 Wheels Gain block
    F_b = 4 * F_b;

    % Summer Operator
    F_fb = F_f - F_b;

    % Define gain and pressure change
    kp = .5; % proportional gain
    dp = kp * F_fb; % change in pressure

    % ABS Module block
    p_b = p_b + dp;
    p_b = min(max(p_b, 0), p_in); % ABS can not increase more than driver input
    
    % Save for next iteration
    p_b1 = p_b;

    % Vehicle block, Acceleration Output
    A_v(ii) = 20 * F_b / m; % includes gain of 20

end

% Create plot
figure;
plot(t, A_v, 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Acceleration (m/s^2)'); title('Linearized Sinusodal Input Output Function');
grid on;
