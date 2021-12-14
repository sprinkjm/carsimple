% model1
% This model represents a more complex kinematic model
function [xdot, ydot, thetadot, deltadot] = ...
    model1(v,theta,delta,omega)

% L is the wheelbase, and is a constant
L = 2.62;

% the deltaT term is taken care of in simulink integration now
xdot = v*cos(theta)*cos(delta);
ydot = v*sin(theta)*cos(delta);
thetadot = (v/L)*sin(delta);
deltadot = omega;

end