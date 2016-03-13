function [lps,raw]=plotme(plotraw)
	bandwidth=500;
	factor=477.66/7.5;%from mobile phone mass cal, 7.5 factor from decimation filter
	[ret,files]=system("ls -t *.wav");
	files=strsplit(files,"\n");
	file=files{1}
	[a,b]=wavread(file);
	time=[1:length(a)]./b;
	a=-a*factor;
	baseline=quantile(a,0.1);
	baseind=find(abs(a-baseline)<(20/2^15));%allow +-20LSB
	baseline=mean(a(baseind));
	a.-=baseline;
	if(exist('plotraw','var'))
		plot(time,a);
		hold on;
	end
	raw=a;
	[c,d]=butter(2,bandwidth/(b/2));
	lps=filtfilt(c,d,a);
	plot(time,lps,'r');
	hold off;
	xlabel "Time (seconds)"
	ylabel "Force (N)"
	grid on
endfunction
