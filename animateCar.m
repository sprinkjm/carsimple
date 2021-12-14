% Author: Jonathan Sprinkle
% Copyright (c) 2008-2015, Arizona Board of Regents
% Reads in a csv formatted .txt file with format
% time,x1,x2,x3,x4
% where
%  time = time in s
%  x1   = x position (xpos) in meters
%  x2   = y position (ypos) in meters
%  x3   = tire angle (tireangle) in radians
%  x4   = heading (heading) in radians
% 
function data = animateCar( stateOutputFileName, stateOutputFileName2 )

% Extra stuff added by JMS to help visualize a vehicle...
L = 2.6187;
xtire = [ 0.4 0.4 -0.4 -0.4 ];
ytire = [ 0.15 -0.15 -0.15 0.15 ];

x = [ (L)+0.3 (L)+0.3 -(max(xtire)/2+0.4) -(max(xtire)/2+0.4) ];
y = [.75 -.75 -.75 .75];

% x = [ (L/2)+0.3 (L/2)+0.3 -((L/2)+0.2) -((L/2)+0.2) ];
% y = [.75 -.75 -.75 .75];
% xtire = [ 0.4 0.4 -0.4 -0.4 ];
% ytire = [ 0.15 -0.15 -0.15 0.15 ];

xlrtire = xtire;
ylrtire = ytire + y(3);
xrrtire = xtire;
yrrtire = ytire + y(4);

% you will need to pass in the path to one of your output files
% in order to make this work
if( nargin < 1 )
   disp 'Usage:'
   disp '   animateCar('''../path/to/output.txt''')'
   return
elseif( nargin > 1 )
    
    % do for file 1
    id1 = fopen(stateOutputFileName);
    data1 = textscan(id1,'%n,%n,%n,%n,%n');
    
    % put each cell into data that we can use
    time1 = data1{1};
    xpos1 = data1{2};
    ypos1 = data1{3};
    tireangle1 = data1{4};
    heading1 = data1{5};
    
    % do for file 2
    id2 = fopen(stateOutputFileName2);
    data2 = textscan(id2,'%n,%n,%n,%n,%n');
    
    % put each cell into data that we can use
    time2 = data2{1};
    xpos2 = data2{2};
    ypos2 = data2{3};
    tireangle2 = data2{4};
    heading2 = data2{5};
    
    figure
    skip = 1/((max(time1)-min(time1))/length(time1));
    skip = ceil(skip/10); % show at tenth of a second intervals
    for i=1:skip:length(xpos1)
        clf
        hold on
        plotPatch( time1,xpos1,ypos1,tireangle1,heading1, i, 0)
        plotPatch( time2,xpos2,ypos2,tireangle2,heading2, i, 0)
    end

    
else

id = fopen(stateOutputFileName);
data = textscan(id,'%n,%n,%n,%n,%n');

% put each cell into data that we can use
time = data{1};
xpos = data{2};
ypos = data{3};
tireangle = data{4};
heading = data{5};

% xlftire = xtire - max(xtire);
% ylftire = ytire + y(1);
% xrftire = xtire - max(xtire);
% yrftire = ytire + y(2);
figure
skip = 1/((max(time)-min(time))/length(time));
skip = ceil(skip/10); % show at tenth of a second intervals
for i=1:skip:length(xpos)
    plotPatch( time,xpos,ypos,tireangle,heading, i)
end

end


function plotPatch( time,xpos,ypos,tireangle,heading,i,clearFig )

  if( nargin ~= 6 )
      if( clearFig == 1 )
          clf
      end
  else
      clf
  end
  hold on
  grid on
  x_tr = xpos(i) + cos(heading(i))*x + sin(heading(i))*y;
y_tr = ypos(i) + sin(heading(i))*x - cos(heading(i))*y;

% do the rear tires
  lrtire_x = xpos(i) + cos(heading(i))*xlrtire + sin(heading(i))*ylrtire;
lrtire_y = ypos(i) + sin(heading(i))*xlrtire - cos(heading(i))*ylrtire;
rrtire_x = xpos(i) + cos(heading(i))*xrrtire + sin(heading(i))*yrrtire;
rrtire_y = ypos(i) + sin(heading(i))*xrrtire - cos(heading(i))*yrrtire;

% rotate front tires by tire angle
xlftire = cos(-tireangle(i))*xtire + sin(-tireangle(i))*ytire;
ylftire = sin(-tireangle(i))*xtire - cos(-tireangle(i))*ytire;
xrftire = cos(-tireangle(i))*xtire + sin(-tireangle(i))*ytire;
yrftire = sin(-tireangle(i))*xtire - cos(-tireangle(i))*ytire;

% perform translation
xlftire = xlftire + L - max(xtire)/2;
ylftire = ylftire + y(1);
xrftire = xrftire + L - max(xtire)/2;
yrftire = yrftire + y(2);

% perform rigid body translation/rotation
lftire_x = xpos(i) + cos(heading(i))*(xlftire) + sin(heading(i))*ylftire;
lftire_y = ypos(i) + sin(heading(i))*(xlftire) - cos(heading(i))*ylftire;
rftire_x = xpos(i) + cos(heading(i))*(xrftire) + sin(heading(i))*yrftire;
rftire_y = ypos(i) + sin(heading(i))*(xrftire) - cos(heading(i))*yrftire;

patch(lrtire_x,lrtire_y,'k')
patch(rrtire_x,rrtire_y,'k')
patch(lftire_x,lftire_y,'k')
patch(rftire_x,rftire_y,'k')
patch(x_tr,y_tr,'w')
plot(xpos(1:i),ypos(1:i));
axis equal

% how much time should we wait before advancing to the next plot point?
if( i > 1 )
    timediff = time(i)-time(i-1);
    pause(timediff)
else
pause(0.1)
end
end

end
