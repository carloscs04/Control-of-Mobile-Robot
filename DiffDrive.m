function [qpunto] = DiffDrive(t,q,v,w)

%This function implements the Differential-Drive kinematic model 
%Giving the state q(k), the inputs v(k) and w(k) at time k, it computes the state at 
%the next time step q(k+1) by solving the differential equations describing the kinematics of
%the robot. 

x=q(1); 
y=q(2); 
theta=q(3);

q1dot=cos(theta)*v;
q2dot=sin(theta)*v;
q3dot = w;


qpunto= [q1dot;q2dot;q3dot];