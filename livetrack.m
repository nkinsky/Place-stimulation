function [pos_out] = livetrack()
% Live tracking in Optitrack test function
   
global pos

try
    c = natnet; % set up NatNet API
    c.connect; % connect to it
    
    % Plot new position every second in 3d coordinates

    pos = [];
    
    figure;
    ax = gca;
    t = timer('TimerFcn', @(x,y)plot_traj(c, ax), 'Period', 0.05, 'ExecutionMode', ...
        'fixedRate', 'TasksToExecute', 200);
    
    start(t)
    
    pause(12)
    
    pos_out = pos;
    clear global
    
catch
    clear global
end

end


function [pos1] = capture_pos(c)

frame = c.getFrame;
pos1 = [frame.RigidBody(1).x, frame.RigidBody(1).y, frame.RigidBody(1).z];

end


function [] = plot_traj(c, ax)
global pos
pos = cat(1, pos, capture_pos(c));
plot3(ax, pos(:,1), pos(:,2), pos(:,3),'b-',...
    pos(end,1), pos(end,2), pos(end,3), 'r*');
drawnow
end