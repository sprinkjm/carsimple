% model2
% This model represents a Dubins kinematic model
function [xdot, ydot, thetadot] = ...
    model2(theta,v,delta,deltaT)

% L is the wheelbase of the vehicle (distance from the center of the front
% two tires to the center of the rear two tires) and is a constant
L = 2.62; % in meters


xdot     = deltaT * v*cos(theta);
ydot     = deltaT * v*sin(theta);
thetadot = deltaT * v*(tan(delta)/L);


end
