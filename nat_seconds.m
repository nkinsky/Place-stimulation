function [time_in_seconds] = nat_seconds(time0, times)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

time_diff = times-time0;

time_in_seconds = time_diff(:,1)*365*24*60*60 + time_diff(:,2)*12*24*60*60+...
    time_diff(:,3)*24*60*60+time_diff(:,4)*60*60 + time_diff(:,5)*60 + ...
    time_diff(:,6);

end

