function [v,w] = Trajectory_Tracking_law(x,y,theta,xr,yr,xdr,ydr,xddr,yddr,v_prev,Ts,kp1,kp2,kd1,kd2,controller_type)
% TRAJECTORY_TRACKING_LAW - Implements two trajectory tracking controllers
%
% Inputs:
%   x, y, theta - Current robot state
%   xr, yr      - Desired position at current time
%   xdr, ydr    - Desired velocity at current time
%   xddr, yddr  - Desired acceleration at current time
%   v_prev      - Previous linear velocity (for integration)
%   Ts          - Sampling time
%   kp1, kp2    - Proportional gains
%   kd1, kd2    - Derivative gains
%   controller_type - 1: De Luca, 2: Feedback Linearization PD
%
% Outputs:
%   v - Linear velocity command
%   w - Angular velocity command

% Default to De Luca if not specified
if nargin < 16
    controller_type = 1;
end

%% Compute Tracking Errors
% Position errors in global frame
ex_global = xr - x;
ey_global = yr - y;

% Desired orientation from velocity
if sqrt(xdr^2 + ydr^2) > 0.001
    theta_d = atan2(ydr, xdr);
else
    theta_d = theta; % If velocity is zero, maintain current orientation
end

% Orientation error (wrapped to [-pi, pi])
e_theta = atan2(sin(theta_d - theta), cos(theta_d - theta));

% Desired velocities
vr = sqrt(xdr^2 + ydr^2);
wr = (yddr * xdr - xddr * ydr) / (xdr^2 + ydr^2 + 1e-6);

%% Controller Selection
switch controller_type
    case 1
        %% DE LUCA CONTROLLER (Nonlinear Trajectory Tracking)
        % Transform errors to robot's local reference frame
        e1 = cos(theta) * ex_global + sin(theta) * ey_global;  % Longitudinal error
        e2 = -sin(theta) * ex_global + cos(theta) * ey_global; % Lateral error
        e3 = e_theta;                                           % Orientation error
        
        % De Luca control law
        % Control gains interpretation:
        % kp1 -> k1 (longitudinal error gain)
        % kp2 -> k2 (lateral error gain)
        % kd1 -> k3 (orientation error gain)
        
        k1 = kp1;
        k2 = kp2; 
        k3 = kd1;
        
        % Linear velocity: feedforward + proportional correction
        v = vr * cos(e3) + k1 * e1;
        
        % Angular velocity: feedforward + lateral and orientation corrections
        % Handle singularity when reference velocity is near zero
        if abs(vr) > 0.01
            w = wr + k2 * vr * e2 + k3 * vr * sin(e3);
        else
            % When reference velocity is near zero, use simpler control
            w = k3 * e3;
            v = k1 * e1;
        end
        
    case 2
        %% FEEDBACK LINEARIZATION PD CONTROLLER
        % This approach linearizes the nonlinear unicycle model and applies
        % PD control to the resulting linear system
        
        % Velocity errors (numerical derivatives using Euler approximation)
        % Approximate current velocities from state
        x_dot = v_prev * cos(theta);
        y_dot = v_prev * sin(theta);
        
        % Velocity errors
        ex_dot = xdr - x_dot;
        ey_dot = ydr - y_dot;
        
        % PD control law for desired accelerations
        % kp1, kp2 -> position gains (Kp_x, Kp_y)
        % kd1, kd2 -> velocity gains (Kd_x, Kd_y)
        
        ux = kp1 * ex_global + kd1 * ex_dot + xddr;
        uy = kp2 * ey_global + kd2 * ey_dot + yddr;
        
        % Feedback linearization: transform (ux, uy) to (v_dot, w)
        % The transformation is:
        %   ux = v_dot * cos(theta) - v * w * sin(theta)
        %   uy = v_dot * sin(theta) + v * w * cos(theta)
        
        % Solve for v_dot (time derivative of linear velocity)
        v_dot = ux * cos(theta) + uy * sin(theta);
        
        % Integrate to get v using simple Euler method
        v = v_prev + v_dot * Ts;
        
        % Solve for w (angular velocity)
        % Avoid division by zero
        if abs(v_prev) > 0.01
            w = (uy * cos(theta) - ux * sin(theta)) / v_prev;
        else
            % Fallback to orientation error correction when v ≈ 0
            % Use a simple P controller for orientation
            K_theta = 3.0; % Angular error gain
            w = K_theta * e_theta;
            % Also add feedforward
            w = w + wr;
        end
        
        % Additional angular correction to prevent drift
        K_theta_correction = 0.5;
        w = w + K_theta_correction * e_theta;
        
    otherwise
        error('Invalid controller type. Use 1 for De Luca or 2 for Feedback Linearization');
end

end

