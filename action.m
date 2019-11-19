function action
disp('Display Figure')
f = figure;
cleanup = onCleanup(@()myCleanupFun(f));
pause(1)
end

function myCleanupFun(f)
disp('Close Figure')
close(f)
end