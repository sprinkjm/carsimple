function [state,inputs] = simulateModel2(inputFile,outputFile)

% Simulates using Model 2, which is a simple kinematics model

% read in the file to the dubins inputs, convert it to dynamics inputs that
% will include acceleration and tire rate of change
if( nargin ~= 2 )
   disp 'Usage:'
   disp '   simulateModel2('''../path/to/input.txt''','''../path/to/output.txt''')'
   return
end

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
u1 = inputs{2}; % velocity
u2 = inputs{3}; % tire angle

state = [];
% outputs are represented by:
% time, x, y, tire(delta), heading(theta)
state_k = [0 0 0 0 0];
state = [state; state_k];
% we can calculate the deltaT for all the calls by looking at the file
% we add the extra at the end to compensate for the diff not matching the
% length of the original matrix
deltaTvector=[diff(time); min(diff(time))];

for k=2:length(time)
    t = time(k);
    % delta_model for model2 is
    % [xdot, ydot, thetadot] = model2(theta,v,delta,deltaT)
    x = state(k-1,2);
    y = state(k-1,3);
    theta=state(k-1,5);
    delta=state(k-1,4);
    deltaT=deltaTvector(k);
    % the value is already multiplied by deltaT inside
    v = u1(k);
    delta = u2(k);
    [xdot,ydot,thetadot] = model2(theta,v,delta,deltaT);
    % time, x, y, tire(delta), heading(theta)
    state_k(1) = t;
    state_k(2) = state_k(2) + xdot;
    state_k(3) = state_k(3) + ydot;
    state_k(4) = delta;
    state_k(5) = state_k(5) + thetadot;
    state = [state; state_k];
end

% now, we write it to the output file
file_out = [outputFile];
dlmwrite(file_out,state,'delimiter',',','precision', 4);


end


