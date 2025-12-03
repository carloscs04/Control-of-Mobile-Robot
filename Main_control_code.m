clear; close all; clc;

%% ROBOT PARAMETERS
r = 0.04445;  % Qbot wheels radius (m)
d = 0.393;    % Qbot wheels-axis length (m)

M = [[r/2 r/2]; [r/d -r/d]];  % Transformation Matrix
Minv = inv(M);                 % Inverse Transformation Matrix

%% SAMPLING TIME
Ts = 0.15;

%% TRAJECTORY PARAMETERS
x0 = 0; y0 = 0; theta0 = 0;
q0 = [x0; y0; theta0];  % Initial conditions

% Scaling factor of the trajectory
eta = 1;
alpha = 4;
k = 0:Ts:2*pi*alpha*2;

xr = eta * sin(k/alpha);           % Trajectory position along x axis
yr = eta * sin(k/(2*alpha));       % Trajectory position along y axis

% Velocity trajectory
xdr = eta * cos(k/alpha) * (1/alpha);              % Trajectory velocity along x axis
ydr = eta * cos(k/(2*alpha)) * (1/(2*alpha));      % Trajectory velocity along y axis

% Acceleration trajectory
xddr = -eta * sin(k/alpha) * (1/alpha) * (1/alpha);           % Trajectory acceleration along x axis
yddr = -eta * sin(k/(2*alpha)) * (1/(2*alpha)) * (1/(2*alpha)); % Trajectory acceleration y axis

% Orientation reference
thetar = atan2(ydr, xdr);  % Trajectory Orientation

% Adjusting Orientation (a part of theta(k) is out of phase)
thetar_diff = diff(thetar);
i1 = 1; i2 = length(thetar);
for i = 1:length(thetar_diff)
    if thetar_diff(i) < -6
        i1 = i + 1;
    elseif thetar_diff(i) > 6
        i2 = i;
    end
end
thetar(i1:i2) = thetar(i1:i2) + 2*pi;

vr = sqrt(xdr.^2 + ydr.^2);                        % Linear velocity of the trajectory
wr = (yddr.*xdr - xddr.*ydr) ./ (xdr.^2 + ydr.^2); % Angular velocity of the trajectory

% Index for scanning the trajectory
K = 1;
Kf = length(xr);

%% CONTROL PARAMETERS
% State Variables
n = 3;
% Input Variables
m = 2;

%% DE LUCA CONTROLLER GAINS
% Controller 1 (De Luca)
kp1_deluca = 2.0;   % k1: Longitudinal error gain
kp2_deluca = 5.0;   % k2: Lateral error gain
kd1_deluca = 3.0;   % k3: Orientation error gain
kd2_deluca = 0.0;   % Not used in De Luca

%% FEEDBACK LINEARIZATION PD CONTROLLER GAINS
% Controller 2 (FL-PD)
kp1_fl = 5.0;   % Kp_x: Proportional gain for x
kp2_fl = 5.0;   % Kp_y: Proportional gain for y
kd1_fl = 3.0;   % Kd_x: Derivative gain for x
kd2_fl = 3.0;   % Kd_y: Derivative gain for y

%% INPUT CONSTRAINTS
wrmax = 10;      % Right wheel limits
wlmax = wrmax;   % Left wheel limits

%% INITIALIZE STORAGE ARRAYS FOR BOTH CONTROLLERS

% De Luca Controller
tt_deluca = [];
qseq_deluca = q0;
vwseq_deluca = [];
v_deluca = vr(1);

% Feedback Linearization Controller
tt_fl = [];
qseq_fl = q0;
vwseq_fl = [];
v_fl = vr(1);

%% TRACKING CONTROL SIMULATION - DE LUCA CONTROLLER
fprintf('=== Running De Luca Controller ===\n');
K = 1;  % Reset trajectory index

for i = 0:Ts:1000
    
    tt_deluca = [tt_deluca i];
    
    % Store state sequence
    x = qseq_deluca(1, end);
    y = qseq_deluca(2, end);
    theta = qseq_deluca(3, end);
    
    % Computing the control law (De Luca: controller_type = 1)
    [v, w] = Trajectory_Tracking_law(x, y, theta, xr(K), yr(K), ...
                                     xdr(K), ydr(K), xddr(K), yddr(K), ...
                                     v_deluca, Ts, ...
                                     kp1_deluca, kp2_deluca, kd1_deluca, kd2_deluca, 1);
    
    % Computing the SATURATION
    [v, w] = unicycle_saturation(wrmax, wlmax, v, w, r, d);
    
    vw = [v; w];
    vwseq_deluca = [vwseq_deluca vw];
    v_deluca = v;  % Update for next iteration
    
    % Feeding the control inputs v(k) and w(k) into the differential drive model
    v1 = vw(1); 
    w1 = vw(2);
    t = 0:0.00001:Ts;
    [t, q] = ode45(@(t,q) DiffDrive(t, q, v1, w1), t, qseq_deluca(:, end));
  
    % Updating the state sequence
    qseq_deluca = [qseq_deluca q(end, :)'];
    
    if K == Kf  % If the trajectory index reaches Kf the simulation is over
        break;
    end
    
    K = K + 1;  % Updating trajectory index
    
end

fprintf('De Luca simulation completed. Steps: %d\n\n', length(tt_deluca));

%% TRACKING CONTROL SIMULATION - FEEDBACK LINEARIZATION CONTROLLER
fprintf('=== Running Feedback Linearization PD Controller ===\n');
K = 1;  % Reset trajectory index

for i = 0:Ts:1000
    
    tt_fl = [tt_fl i];
    
    % Store state sequence
    x = qseq_fl(1, end);
    y = qseq_fl(2, end);
    theta = qseq_fl(3, end);
    
    % Computing the control law (FL-PD: controller_type = 2)
    [v, w] = Trajectory_Tracking_law(x, y, theta, xr(K), yr(K), ...
                                     xdr(K), ydr(K), xddr(K), yddr(K), ...
                                     v_fl, Ts, ...
                                     kp1_fl, kp2_fl, kd1_fl, kd2_fl, 2);
    
    % Computing the SATURATION
    [v, w] = unicycle_saturation(wrmax, wlmax, v, w, r, d);
    
    vw = [v; w];
    vwseq_fl = [vwseq_fl vw];
    v_fl = v;  % Update for next iteration
    
    % Feeding the control inputs v(k) and w(k) into the differential drive model
    v1 = vw(1); 
    w1 = vw(2);
    t = 0:0.00001:Ts;
    [t, q] = ode45(@(t,q) DiffDrive(t, q, v1, w1), t, qseq_fl(:, end));
  
    % Updating the state sequence
    qseq_fl = [qseq_fl q(end, :)'];
    
    if K == Kf  % If the trajectory index reaches Kf the simulation is over
        break;
    end
    
    K = K + 1;  % Updating trajectory index
    
end

fprintf('Feedback Linearization simulation completed. Steps: %d\n\n', length(tt_fl));

%% COMPUTE TRACKING ERRORS

% De Luca errors
N_deluca = min(length(tt_deluca), length(xr));
errors_deluca = zeros(3, N_deluca);
for i = 1:N_deluca
    errors_deluca(1, i) = xr(i) - qseq_deluca(1, i);
    errors_deluca(2, i) = yr(i) - qseq_deluca(2, i);
    errors_deluca(3, i) = atan2(sin(thetar(i) - qseq_deluca(3, i)), ...
                                cos(thetar(i) - qseq_deluca(3, i)));
end

% FL-PD errors
N_fl = min(length(tt_fl), length(xr));
errors_fl = zeros(3, N_fl);
for i = 1:N_fl
    errors_fl(1, i) = xr(i) - qseq_fl(1, i);
    errors_fl(2, i) = yr(i) - qseq_fl(2, i);
    errors_fl(3, i) = atan2(sin(thetar(i) - qseq_fl(3, i)), ...
                            cos(thetar(i) - qseq_fl(3, i)));
end

%% PERFORMANCE METRICS
fprintf('=== PERFORMANCE METRICS ===\n\n');

% De Luca metrics
eucl_error_deluca = sqrt(errors_deluca(1,:).^2 + errors_deluca(2,:).^2);
mae_deluca = [mean(abs(errors_deluca(1,:))); 
              mean(abs(errors_deluca(2,:))); 
              mean(abs(errors_deluca(3,:)))];
rmse_deluca = [sqrt(mean(errors_deluca(1,:).^2)); 
               sqrt(mean(errors_deluca(2,:).^2)); 
               sqrt(mean(errors_deluca(3,:).^2))];
max_error_deluca = [max(abs(errors_deluca(1,:))); 
                    max(abs(errors_deluca(2,:))); 
                    max(abs(errors_deluca(3,:)))];
mae_pos_deluca = mean(eucl_error_deluca);
rmse_pos_deluca = sqrt(mean(eucl_error_deluca.^2));

fprintf('De Luca Controller:\n');
fprintf('  Position Error (MAE): %.4f m\n', mae_pos_deluca);
fprintf('  Position Error (RMSE): %.4f m\n', rmse_pos_deluca);
fprintf('  Max Position Error: %.4f m\n', max(eucl_error_deluca));
fprintf('  Orientation Error (MAE): %.4f rad (%.2f deg)\n', mae_deluca(3), rad2deg(mae_deluca(3)));
fprintf('  Orientation Error (RMSE): %.4f rad (%.2f deg)\n\n', rmse_deluca(3), rad2deg(rmse_deluca(3)));

% FL-PD metrics
eucl_error_fl = sqrt(errors_fl(1,:).^2 + errors_fl(2,:).^2);
mae_fl = [mean(abs(errors_fl(1,:))); 
          mean(abs(errors_fl(2,:))); 
          mean(abs(errors_fl(3,:)))];
rmse_fl = [sqrt(mean(errors_fl(1,:).^2)); 
           sqrt(mean(errors_fl(2,:).^2)); 
           sqrt(mean(errors_fl(3,:).^2))];
max_error_fl = [max(abs(errors_fl(1,:))); 
                max(abs(errors_fl(2,:))); 
                max(abs(errors_fl(3,:)))];
mae_pos_fl = mean(eucl_error_fl);
rmse_pos_fl = sqrt(mean(eucl_error_fl.^2));

fprintf('Feedback Linearization PD Controller:\n');
fprintf('  Position Error (MAE): %.4f m\n', mae_pos_fl);
fprintf('  Position Error (RMSE): %.4f m\n', rmse_pos_fl);
fprintf('  Max Position Error: %.4f m\n', max(eucl_error_fl));
fprintf('  Orientation Error (MAE): %.4f rad (%.2f deg)\n', mae_fl(3), rad2deg(mae_fl(3)));
fprintf('  Orientation Error (RMSE): %.4f rad (%.2f deg)\n\n', rmse_fl(3), rad2deg(rmse_fl(3)));

%% PLOTTING
fprintf('Generating plots...\n');

%% Figure 1: Trajectory Comparison
figure('Name', 'Trajectory Comparison', 'Position', [100 100 1000 800]);
plot(xr, yr, 'k--', 'LineWidth', 2.5, 'DisplayName', 'Desired Trajectory');
hold on;
plot(qseq_deluca(1,:), qseq_deluca(2,:), 'b-', 'LineWidth', 1.8, 'DisplayName', 'De Luca');
plot(qseq_fl(1,:), qseq_fl(2,:), 'r-', 'LineWidth', 1.8, 'DisplayName', 'FL-PD');
plot(q0(1), q0(2), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
xlabel('X Position (m)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Y Position (m)', 'FontSize', 13, 'FontWeight', 'bold');
title('Trajectory Tracking Comparison', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 12);
grid on;
axis equal;

%% Figure 2: Euclidean Distance Error
figure('Name', 'Position Error', 'Position', [150 150 1000 500]);
plot(tt_deluca(1:N_deluca), eucl_error_deluca, 'b-', 'LineWidth', 2, 'DisplayName', 'De Luca');
hold on;
plot(tt_fl(1:N_fl), eucl_error_fl, 'r-', 'LineWidth', 2, 'DisplayName', 'FL-PD');
xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Euclidean Error (m)', 'FontSize', 13, 'FontWeight', 'bold');
title('Position Tracking Error', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 12);
grid on;

%% Figure 3: X and Y Errors
figure('Name', 'X and Y Errors', 'Position', [200 200 1200 600]);

subplot(2,1,1);
plot(tt_deluca(1:N_deluca), errors_deluca(1,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'De Luca');
hold on;
plot(tt_fl(1:N_fl), errors_fl(1,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
xlabel('Time (s)', 'FontSize', 12);
ylabel('X Error (m)', 'FontSize', 12);
title('X Position Error', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;

subplot(2,1,2);
plot(tt_deluca(1:N_deluca), errors_deluca(2,:), 'b-', 'LineWidth', 1.5, 'DisplayName', 'De Luca');
hold on;
plot(tt_fl(1:N_fl), errors_fl(2,:), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Y Error (m)', 'FontSize', 12);
title('Y Position Error', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;

%% Figure 4: Orientation Error
figure('Name', 'Orientation Error', 'Position', [250 250 1000 500]);
plot(tt_deluca(1:N_deluca), rad2deg(errors_deluca(3,:)), 'b-', 'LineWidth', 2, 'DisplayName', 'De Luca');
hold on;
plot(tt_fl(1:N_fl), rad2deg(errors_fl(3,:)), 'r-', 'LineWidth', 2, 'DisplayName', 'FL-PD');
xlabel('Time (s)', 'FontSize', 13, 'FontWeight', 'bold');
ylabel('Orientation Error (deg)', 'FontSize', 13, 'FontWeight', 'bold');
title('Orientation Tracking Error', 'FontSize', 15, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 12);
grid on;

%% Figure 5: Control Inputs
figure('Name', 'Control Inputs', 'Position', [300 300 1200 600]);

% Make sure dimensions match
N_ctrl_deluca = min(length(tt_deluca)-1, size(vwseq_deluca, 2));
N_ctrl_fl = min(length(tt_fl)-1, size(vwseq_fl, 2));

subplot(2,1,1);
plot(tt_deluca(1:N_ctrl_deluca), vwseq_deluca(1,1:N_ctrl_deluca), 'b-', 'LineWidth', 1.5, 'DisplayName', 'De Luca');
hold on;
plot(tt_fl(1:N_ctrl_fl), vwseq_fl(1,1:N_ctrl_fl), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Linear Velocity v (m/s)', 'FontSize', 12);
title('Linear Velocity Command', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;

subplot(2,1,2);
plot(tt_deluca(1:N_ctrl_deluca), vwseq_deluca(2,1:N_ctrl_deluca), 'b-', 'LineWidth', 1.5, 'DisplayName', 'De Luca');
hold on;
plot(tt_fl(1:N_ctrl_fl), vwseq_fl(2,1:N_ctrl_fl), 'r-', 'LineWidth', 1.5, 'DisplayName', 'FL-PD');
xlabel('Time (s)', 'FontSize', 12);
ylabel('Angular Velocity ω (rad/s)', 'FontSize', 12);
title('Angular Velocity Command', 'FontSize', 13, 'FontWeight', 'bold');
legend('Location', 'best', 'FontSize', 11);
grid on;

%% Animation
fprintf('Creating animation...\n');
animate_controllers(xr, yr, qseq_deluca, qseq_fl, tt_deluca, tt_fl);

fprintf('\nAll plots and animations complete!\n');

%% ANIMATION FUNCTION
function animate_controllers(xr, yr, qseq_deluca, qseq_fl, tt_deluca, tt_fl)
    figure('Name', 'Robot Tracking Animation', 'Position', [100 100 1400 700]);
    set(gcf, 'Color', 'w');  % White background
    
    robot_size = 0.15;
    skip_frames = 2;  % Show every 2nd frame for smoother animation
    
    N = min([length(xr), size(qseq_deluca,2), size(qseq_fl,2)]);
    
    fprintf('Starting animation with %d frames (showing every %d frames)...\n', N, skip_frames);
    
    for i = 1:skip_frames:N
        clf;
        
        % Left subplot: Full trajectory view
        subplot(1,2,1);
        plot(xr, yr, 'k--', 'LineWidth', 2.5);
        hold on;
        plot(qseq_deluca(1,1:i), qseq_deluca(2,1:i), 'b-', 'LineWidth', 2);
        plot(qseq_fl(1,1:i), qseq_fl(2,1:i), 'r-', 'LineWidth', 2);
        
        % Draw robots with enhanced visibility
        draw_robot(qseq_deluca(1,i), qseq_deluca(2,i), qseq_deluca(3,i), robot_size, 'b');
        draw_robot(qseq_fl(1,i), qseq_fl(2,i), qseq_fl(3,i), robot_size, 'r');
        
        % Add labels for robots
        text(qseq_deluca(1,i), qseq_deluca(2,i)+0.15, 'De Luca', ...
             'Color', 'b', 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center');
        text(qseq_fl(1,i), qseq_fl(2,i)-0.15, 'FL-PD', ...
             'Color', 'r', 'FontWeight', 'bold', 'FontSize', 10, 'HorizontalAlignment', 'center');
        
        xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
        if i <= length(tt_deluca)
            title(sprintf('Full Trajectory View | Time: %.2f s', tt_deluca(i)), ...
                  'FontSize', 13, 'FontWeight', 'bold');
        end
        legend('Desired', 'De Luca Path', 'FL-PD Path', 'Location', 'northoutside', ...
               'Orientation', 'horizontal', 'FontSize', 11);
        grid on;
        axis equal;
        xlim([min(xr)-0.3 max(xr)+0.3]);
        ylim([min(yr)-0.3 max(yr)+0.3]);
        
        % Right subplot: Zoomed view
        subplot(1,2,2);
        zoom_range = 0.4;
        
        % Plot trajectory segment
        idx_range = max(1,i-30):min(i+10, length(xr));
        plot(xr(idx_range), yr(idx_range), 'k--', 'LineWidth', 2.5);
        hold on;
        
        % Plot robot paths
        path_range = max(1,i-20):i;
        plot(qseq_deluca(1,path_range), qseq_deluca(2,path_range), 'b-', 'LineWidth', 2);
        plot(qseq_fl(1,path_range), qseq_fl(2,path_range), 'r-', 'LineWidth', 2);
        
        % Draw robots
        draw_robot(qseq_deluca(1,i), qseq_deluca(2,i), qseq_deluca(3,i), robot_size, 'b');
        draw_robot(qseq_fl(1,i), qseq_fl(2,i), qseq_fl(3,i), robot_size, 'r');
        
        % Draw desired position
        plot(xr(i), yr(i), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'y', ...
             'LineWidth', 2, 'DisplayName', 'Target');
        
        xlabel('X (m)', 'FontSize', 12, 'FontWeight', 'bold');
        ylabel('Y (m)', 'FontSize', 12, 'FontWeight', 'bold');
        title('Zoomed View (Following Robots)', 'FontSize', 13, 'FontWeight', 'bold');
        grid on;
        axis equal;
        xlim([xr(i)-zoom_range xr(i)+zoom_range]);
        ylim([yr(i)-zoom_range yr(i)+zoom_range]);
        
        drawnow;
        pause(0.05);  % Adjust speed (lower = faster)
    end
    
    fprintf('Animation complete!\n');
end

function draw_robot(x, y, theta, size, color)
    % Draw robot body (thicker line)
    body_x = [x - size/2*cos(theta), x + size/2*cos(theta)];
    body_y = [y - size/2*sin(theta), y + size/2*sin(theta)];
    plot(body_x, body_y, color, 'LineWidth', 6);
    
    % Draw direction indicator (arrow)
    dir_x = x + size*cos(theta);
    dir_y = y + size*sin(theta);
    plot([x dir_x], [y dir_y], color, 'LineWidth', 3);
    
    % Draw robot "head" (filled circle)
    plot(dir_x, dir_y, [color 'o'], 'MarkerSize', 10, 'MarkerFaceColor', color, ...
         'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % Draw robot center
    plot(x, y, [color 'o'], 'MarkerSize', 6, 'MarkerFaceColor', 'w', ...
         'MarkerEdgeColor', color, 'LineWidth', 2);
end