% generateTests
function generateInputs()

steerMax = pi/6;
omega = 0.05;
inputs = [];
for t=0:0.1:5
    inputs = [inputs; t 1 steerMax*sin(omega*t)];
end
dlmwrite('input2.txt',inputs,'delimiter',',','precision', 4);


inputs = [];
for t=0:0.1:5
    inputs = [inputs; t 1 1/steerMax*sin(omega*t)];
end
dlmwrite('input3.txt',inputs,'delimiter',',','precision', 4);

% generate a few files for test cases
waypointTestGenerator(30,0,'input4',12, 0);
waypointTestGenerator(30,30,'input5',12, 0);
waypointTestGenerator(-10,30,'input6',30, 1);

end

% a naive controller 
% 
% it calculates new inputs based on current input and state, as well as
% the error function
function u_next = g( waypoint, state, u )
    maxu1 = 30;
    minu1 = 0;
    maxu2 = pi/6;
    minu2 = -pi/6;

    % tire angle gain
    K2 = 0.1;
    % velocity gain
    K1 = 0.5;
    
    u_next = u;
    yerr = waypoint(2)-state(2);
    xerr = waypoint(1)-state(1);
    xyerr = sqrt(yerr^2 + xerr^2);
    
    % speed up if we are far away
    if( xyerr < 2 )
        u_next(1) = 0.9*u(1);
    else
        u_next(1) = max(min(K1*xyerr,maxu1),minu1);
    end
    % if the target is to our left, or right, we change our tire
    headerr = (state(4) - atan2(yerr,xerr)) - state(3);
    u_next(2) = (max(min(-K2*headerr,maxu2),minu2));
end

function xdot = carfcn( x, u, dt )
    % vehicle wheelbase
    L = 2.6187;    
    xdot = x;
    xdot(1) = dt * u(1)*cos(x(3))*cos(x(4));    % x position
    xdot(2) = dt * u(1)*cos(x(3))*sin(x(4));    % y position
    xdot(3) = dt * u(2);                        % tire angle
    xdot(4) = dt * u(1)*(1/L)*sin(x(3));        % heading
end

function waypointTestGenerator( xpos, ypos, testname, time, random )
    inputs = [];
    state = [0 0 0 0];
    waypoint = [xpos ypos 0 0];
    errarray = [];
    states = [];
    u = [0 0];
    
    t = [0]; 
    while( t(end) <= time )
        if( random == 0 )
            t = [t t(end)+(0.1)];
        else
            t = [t t(end)+(0.05 + rand*0.1)];
        end
    end
    
    for i = 1:length(t)
        err = waypoint - state;
        errarray = [errarray; err];
        u = g(waypoint,state,u);
        if( t > 0 )
            state = state + carfcn(state,u,t(i)-inputs(end,1));
        else
            state = state + carfcn(state,u,0.1);
        end
        states = [states; state];
        inputs = [inputs; t(i) u(1) u(2)];
    end
    file = [testname '.txt'];
    dlmwrite(file,inputs,'delimiter',',','precision', 4);
    figure
    % plot(errarray)
    % plot(inputs(:,1),states(:,1),inputs(:,1),errarray(:,1))
    plot(errarray(:,1:2))
    title(testname)
end