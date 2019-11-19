function [] = zone_to_ttl(debug)
% Function to ID track and zones to trigger TTL out pulses

if nargin < 1
    debug = false;
end
clear global
global D2value
global pos
D2value = 0;

run_time = 60*60; %seconds
SR = 20; %Hz

% Construct pos vector to keep track of last 0.25 seconds
nquarter = ceil(SR/4); % #samples in a quarter second
pos = repmat([0 0 -500], nquarter, 1); % Start pos with z-position waaaay off

% Make sure track is aligned with z axis in optitrack calibration first!
% Connect to optitrack
trackobj = natnet;
trackobj.connect;

% set up arduino/test if arduino not working
if ~debug
    try
        a = arduino;
        configurePin(a,'D2','DigitalOutput');
    catch
        disp('error connecting to arduino - running in debug mode')
        debug = true;
    end
end

% set up window sanity check
hf = figure; ax = gca;
imagesc(ax, 1);
colormap(ax,[1 0 0])
ht = text(ax, 1, 1, 'OFF', 'FontSize', 50, 'HorizontalAlignment', 'center');

% Make aware running in DEBUG mode, set arduino object to nan.
if debug
    a = nan;
    disp('RUNNING IN DEBUG MODE!!!')
end

%Get track ends
input('Put rigid body at start of the track. Hit enter when done')
capture_pos(trackobj);
start_pos = pos(end,:);
% start_pos = [-0.2557 0.1353 -3.6050]; % For debugging only

input('Put rigid body at end of the track. Hit enter when done')
capture_pos(trackobj);
end_pos = pos(end,:);
% end_pos = [-0.3754 0.2064 3.8861]; % For debugging only

% Calculate track distance along the z-axis (must set up in OptiTrack
% first!)
% to-do - calculate angle of track, get long-axis, find closest point on a
% line, use that!
track_dist = end_pos(3) - start_pos(3);

% Calculate stim zone - middle of the track!
ttl_zone = [start_pos(3) + track_dist/3, start_pos(3) + track_dist*2/3];
disp(['track start = ' num2str(ttl_zone(1),'%0.2g')])
disp(['track end = ' num2str(ttl_zone(2),'%0.2g')])

% Start timer to check every SR Hz if rat is in the stim zone.
t = timer('TimerFcn', @(x,y)zone_detect(trackobj, a, ax, ht, ttl_zone), 'Period', 1/SR, ...
    'ExecutionMode', 'fixedRate', 'TasksToExecute', SR*run_time); %, ...
%     'StopFcn', @(x,y)trigger_off(a, ax, ht, pos));

% Create cleanup function
cleanup = onCleanup(@()myCleanupFun(t, a, ax, ht, pos));

% start function!
start(t)
figure(hf); % bring figure to front

pause(run_time+3); % Don't get out of function until timer is done running

end

%% Capture live position
function [delta_pos] = capture_pos(c)
% adjust this to get delta pos from 0.25 sec prior!!!
global pos

frame = c.getFrame; % get frame

% Add position to bottom of position tally
pos = [pos; frame.RigidBody(1).x, frame.RigidBody(1).y, frame.RigidBody(1).z];

% get change in position from  0.25 seconds ago
delta_pos = pos(end,:) - pos(1,:);

% update pos to chop off most distant time point
pos = pos(2:end,:);

end

%% Detect if in zone and trigger
function [] = zone_detect(c, a, ax, ht, ttl_zone)
global pos
delta_pos = capture_pos(c); % get position
pos_curr = pos(end,:);

% Turn TTL off if rat's position has not changed at all (most likely
% optitrack can't find it)
if all(delta_pos == 0)
    trigger_off(a, ax, ht, pos_curr)
    
else % Logic to trigger is the rat is in the appropriate zone below
%     Send D2 to 5V if in zone and currently at 0
    if pos_curr(3) > ttl_zone(1) && pos_curr(3) < ttl_zone(2)
        trigger_on(a, ax, ht, pos_curr)
        
%         Send D2 to 0V if outside of zone and currently at 5V
    elseif (pos_curr(3) <= ttl_zone(1)) || (pos_curr(3) >= ttl_zone(2))
        trigger_off(a, ax, ht, pos_curr)
        
    end
    
end

end

%% Turn on LED/screen
function [] = trigger_on(a, ax, ht, pos_curr)

global D2value
D2value = 1;
text_append = '';

if ~isnan(a)
    writeDigitalPin(a,'D2',D2value)
elseif isnan(a)
    text_append = ['-' num2str(pos_curr(3), '%0.2g')];
end

colormap(ax,[0 1 0])
ht.String = ['ON' text_append];

end

%% Turn off LED/screen
function [] = trigger_off(a, ax, ht, pos_curr)

global D2value
D2value = 0;
text_append = '';

if ~isnan(a)
    writeDigitalPin(a,'D2',D2value)
elseif isnan(a)
    text_append = ['-' num2str(pos_curr(3), '%0.2g')];
end

colormap(ax,[1 0 0])
ht.String = ['OFF' text_append];

end

%% Clean up function to make sure trigger gets turned off, timer stopped, global
% vars cleared if function stops for any reason!!!
function myCleanupFun(t, a, ax, ht, pos_curr)

try
    trigger_off(a, ax, ht, pos_curr)
    stop(t)
    clear global
    close(ax.Parent(1))
    disp('cleanup function ran!')
catch
    disp('error running cleanup function - clear all global varibles manually!')
end

end