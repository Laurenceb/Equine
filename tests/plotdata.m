function n=plotdata(location_root,optarg)
	[n,m]=system(["ls -tr '",location_root,"'"]);
	if(n)
		return;		%failure
	end
	s=strsplit(m,"\n");
	folder=s{end-1};	%the most recent folder
	folder=[location_root,"/",folder,"/",folder];% the most recent recording directory
	ecg=[folder,".wav"];
	gps=[folder,"_gps.wav"];
	imu=[folder,"_imu.wav"];
	[ecgd,sprecg]=wavread(ecg);
	[gpsd,sprgps]=wavread(gps);
	[imud,sprimu]=wavread(imu);
	%process and plot the ECG data
	gain=4;			%gain setting for the ECG
	sig_to_mv=1200/gain;
	ecgd(find(abs(ecgd)>=0.999969))=NA;%all error code stuff blanked out
	ecgd=((ecgd.+circshift(ecgd,2))./2).*sig_to_mv;%remove the lead off signal
	ecgt=[0:length(ecgd)-1]./sprecg;
	subplot(1,1,1);
	clf;
	plot(ecgt,ecgd);
	traces={"RA","LA","LL","V1","V2","V3","V4","V5"};
	xlabel("Time (seconds)");
	ylabel("ECG (mV)");
	legend(traces);
	%process the gps and plot location
	if(max(abs(gpsd(1:end-1,1:2)))>0)
		indx=min(find(abs(gpsd(:,1))>0));	 
		latlong=gpsd(indx+1,1:2).*2^(15+16);
		lat=gpsd(indx,1)*2^15;
		if(lat<0)
			lat+=2^16;
		end
		long=gpsd(indx,2)*2^15;
		if(long<0)
			long+=2^16;
		end
		latlong+=[lat,long];
		latlong*=1e-7;		%now it is in units of degrees
		altitude=gpsd(:,3).*(2^15).*1e-2;
		altitude(find(altitude<0))+=0.01*2^16;%this wrap around allows operation up to 2150 feet above mean sea level
		speed=sqrt(gpsd(:,4).^2.+gpsd(:,5).^2).*(2^15).*0.01;%meters per second
		speedkmh=speed*3.6;	%km/h speed
		printf("Latitude:%.4fN, Longitude:%.4fE, Altitude %.1f(msl)\n",latlong(1),latlong(2),altitude(indx));fflush(stdout);
		figure();
		subplot(2,1,1);		%to fit everything onto the plot
		plot(gpsd(indx+2:end,2).*2^15,gpsd(indx+2:end,1).*2^15);%position in meters
		xlabel("Easting(m)");
		ylabel("North(m)");
		subplot(2,1,2);
		gpst=[indx+2:length(gpsd)]./sprgps;
		plot(gpst,speedkmh(indx+2:end));
		xlabel("Time (seconds)");
		ylabel("Speed (km/h)");
		if(nargin>1)
			nsats=gpsd(indx+2:end,6).*2^15;
			figure();
			subplot(1,1,1);
			nsats_=floor(nsats./256);
			code=floor((nsats.-(nsats_*256))./16);
			plot(gpst,nsats_);
			hold on;
			plot(gpst,code,'r');
			legend("Number of sats","Button code");
			hold off;
		end
	end
	n=0;
endfunction
