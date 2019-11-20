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
        % First close any existing instances of the arduino
        % This doesn't seem to work - arduino can be open in the main
        % workspace but not detected here...
        try % this will error out if there isn't an arduino already connected...
            fclose(instrfindall);
        end
        try
            delete(instrfindall);
        end
            
        % now connect
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
input('Put rigid body at start of the track. Hit enter when done');
capture_pos(trackobj);
start_pos = pos(end,:);
% start_pos = [-0.2557 0.1353 -3.6050]; % For debugging only

input('Put rigid body at end of the track. Hit enter when done');
capture_pos(trackobj);
end_pos = pos(end,:);
% end_pos = [-0.3754 0.2064 3.8861]; % For debugging only

% Calculate track distance along the z-axis (must set up in OptiTrack
% first! ( Assumes y is up, x-z plane is on ground! Make sure "optitrack
% streaming engine tab has Up Axis = y!!!)

% calculate angle of track for transformation to track coords
center = mean([start_pos; end_pos]);
theta = atan2(end_pos(3)-start_pos(3), end_pos(1) - start_pos(1));
track_length = pdist2(end_pos, start_pos);

% Calculate stim zone - middle of the track! Double check!!!
ttl_zone = [-1/3, 1/3]*track_length/2;

% Below is code if track is aligned with z-axis perfectly!!!
track_zdist = end_pos(3) - start_pos(3);
ttl_zzone = [start_pos(3) + track_zdist/3, start_pos(3) + track_zdist*2/3];

disp(['zone start = ' num2str(ttl_zone(1),'%0.2g')])
disp(['zone end = ' num2str(ttl_zone(2),'%0.2g')])

% Start timer to check every SR Hz if rat is in the stim zone.
t = timer('TimerFcn', @(x,y)zone_detect(trackobj, a, ax, ht, ttl_zone, theta, center), ...
    'Period', 1/SR, ...
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
function [] = zone_detect(c, a, ax, ht, ttl_zone, theta, center)
global pos
delta_pos = capture_pos(c); % get position
pos_curr = pos(end,:);

pos_s = cart_to_track(pos_curr, theta, center);

% Turn TTL off if rat's position has not changed at all (most likely
% optitrack can't find it)
if all(delta_pos == 0)
    trigger_off(a, ax, ht, pos_curr)
    
else % Logic to trigger is the rat is in the appropriate zone below
%     Send D2 to 5V if in zone and currently at 0
    if (pos_s > ttl_zone(1)) && (pos_s < ttl_zone(2)) %pos_curr(3) > ttl_zone(1) && pos_curr(3) < ttl_zone(2)
        trigger_on(a, ax, ht, pos_s)
        
%         Send D2 to 0V if outside of zone and currently at 5V
    elseif (pos_s <= ttl_zone(1)) || (pos_s >= ttl_zone(2)) %(pos_curr(3) <= ttl_zone(1)) || (pos_curr(3) >= ttl_zone(2))
        trigger_off(a, ax, ht, pos_s)
        
    end
    
end

end

%% Turn on LED/screen
function [] = trigger_on(a, ax, ht, pos_curr)

global D2value
D2value = 1;
% text_append = '';

if length(pos_curr) == 3
    pos_use = pos_curr(3);
else
    pos_use = pos_curr;
end

if isobject(a)
    writeDigitalPin(a,'D2',0) % OCSlite1 only seems to trigger when it detects an off->on tranisition
    writeDigitalPin(a,'D2',D2value)
end
text_append = ['-' num2str(pos_use, '%0.2g')];

colormap(ax,[0 1 0])
ht.String = ['ON' text_append];

end

%% Turn off LED/screen
function [] = trigger_off(a, ax, ht, pos_curr)

global D2value
D2value = 0;
% text_append = '';

if length(pos_curr) == 3
    pos_use = pos_curr(3);
else
    pos_use = pos_curr;
end

if isobject(a)
    writeDigitalPin(a,'D2',D2value)
end
text_append = ['-' num2str(pos_use, '%0.2g')];

colormap(ax,[1 0 0])
ht.String = ['OFF' text_append];

end

%% Convert cartesian position to track length
function [s] = cart_to_track(pos_curr, theta, center)
% s = position on track

x = pos_curr(1); y = pos_curr(2); z = pos_curr(3);
xmid = center(1); ymid = center(2); zmid = center(3);

% calculate s two different ways
s1 = (z - zmid)/sin(theta);
s2 = (x - xmid)/cos(theta);

% Make sure you aren't dividing by zero for your calculation.
cos_lims = [-pi(), -3*pi()/4; -pi()/4, pi()/4; 3*pi()/4, pi()];
sin_lims = [-3*pi()/4 -pi()/4; pi()/4 3*pi()/4];
if any(cos_lims(:,1) <= theta & theta < cos_lims(:,2))
    s = s2;
elseif any(sin_lims(:,1) <= theta & theta < sin_lims(:,2))
    s = s1;
end

end

%% Clean up function to make sure trigger gets turned off, timer stopped, global
% vars cleared if function stops for any reason!!!
function myCleanupFun(t, a, ax, ht, pos_curr)

try
    trigger_off(a, ax, ht, pos_curr)
    clear a
    try
        fclose(instrfindall)
        delete(instrfindall)
    end
    stop(t)
    clear global

    close(ax.Parent(1))
    disp('cleanup function ran!')
catch
    disp('error running cleanup function - clear all global variables manually!')
end

end

