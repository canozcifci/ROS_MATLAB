
r = rateControl(100);
i = 0;
reset(r)
while(1)
	time = r.TotalElapsedTime;
	fprintf('Iteration: %d - Time Elapsed: %f\n',i,time)
    i = i+1;
	waitfor(r);
end
