function [state,inputs] = simulateModel1(inputFile,outputFile)

% Simulates using Model 1, which is a complex dynamical model

% read in the file to the dubins inputs, convert it to dynamics inputs that
% will include acceleration and tire rate of change
if( nargin ~= 2 )
   disp 'Usage:'
   disp '   simulateModel1('''../path/to/input.txt''','''../path/to/output.txt''')'
   return
end

% maximum values for speed and steering
maxu1 = 30;
minu1 = 0;
deltaMax = pi/6;

% the file input values are
% u(1) = time, velocity, tire_angle
id = fopen(inputFile);
inputs = textscan(id,'%n,%n,%n');

% now, execute the dynamics based on these inputs; assume an initial
% time = 0
% velocity = 0
% theta = 0
% tire angle = 0
% put each cell into data that we can use
time = inputs{1};
u1_dubins = inputs{2}; % velocity
u2_dubins = inputs{3}; % tire angle

% convert velocity to accelerations
% function [xdot, ydot, vdot, thetadot, phidot, deltadot] = ...
%     model1(v,theta,phi,delta,a,omega,deltaT)
% inputs = [a,omega], acceleration, rotation of steering speed
% states = [x,y,v,theta,phi(theta_dot),delta]

dt = diff(time);

% this model uses velocities
u1 = u1_dubins;
% convert from tire angles to tire rates
u2 = [0; diff(u2_dubins)./dt];

state = [];
% states are represented by:
% [time,x,y,theta,delta]
state_k = [0 0 0 0 0];
state = [state; state_k];
% we can calculate the dT for all the calls by looking at the file
% we add the extra at the end to compensate for the diff not matching the
% length of the original matrix
dTvector=[diff(time); min(diff(time))];

% We start with default values:


for k=2:length(time)
    t = time(k);
    % delta_model for model2 is
    % [xdot, ydot, vdot, thetadot, phidot, deltadot] = model1(v,theta,phi,delta,a,omega,deltaT)
    x = state(k-1,2);
    y = state(k-1,3);
    theta=state(k-1,4);
    delta=state(k-1,5);
    dT=dTvector(k);
    % the value is already multiplied by deltaT inside
    v = u1(k);
    omega = u2(k);
    [xdot, ydot, thetadot, deltadot] = model1(v,theta,delta,omega);
    % t, x,y,v,theta,phi(theta_dot),delta
    state_k(1) = t;
    state_k(2) = state_k(2) + dT * xdot;
    state_k(3) = state_k(3) + dT * ydot;
    state_k(4) = mod((state_k(4) + dT * thetadot),2*pi);
    state_k(5) = min(deltaMax,max(-deltaMax,state_k(5) + dT * deltadot));
    state = [state; state_k];
end

file_out = [outputFile];
% now, we write it to the output file, but only some of the states:
%  time = time in s
%  x1   = x position (xpos) in meters
%  x2   = y position (ypos) in meters
%  x3   = tire angle (tireangle) in radians
%  x4   = heading (heading) in radians
% these are elements 1,2,3,5,4in the state matrix
dlmwrite(file_out,state(:,[1:3,5,4]),'delimiter',',','precision', 4);



end


