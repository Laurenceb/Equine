function [n,rawdata,filtdata]=plotdata(location_root,optarg)
	[n,m]=system(["ls -tr '",location_root,"'"]);
	if(n)
		return;		%failure
	end
	s=strsplit(m,"\n");
	folder=s{end-1};	%the most recent folder
	printf("%s\n",folder);fflush(stdout);
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
	rawdata=ecgd;		%raw data in mv is returned
	[a,b]=butter(2,2*30/sprecg);%30hz low pass
	filtdata=rawdata;
	filtdata(isna(filtdata))=0;
	filtdata=filtfilt(a,b,filtdata);
	[a,b]=butter(2,2*0.45/sprecg,'high');%0.45hz high pass
	filtdata=filtfilt(a,b,filtdata);
	ecgt=[0:length(ecgd)-1]./sprecg;
	filtdata=[ecgt',filtdata];
	subplot(1,1,1);
	clf;
	plot(ecgt,ecgd);
	traces={"RA","LA","LL","V1","V2","V3","V4","V5"};
	xlabel("Time (seconds)");
	ylabel("ECG (mV)");
	legend(traces);
	set(gca(),'ycolor','b');
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
		set(gca(),'ycolor','b');
		subplot(2,1,2);
		gpst=[indx+2:length(gpsd)]./sprgps;
		plot(gpst,speedkmh(indx+2:end));
		xlabel("Time (seconds)");
		ylabel("Speed (km/h)");
		set(gca(),'ycolor','b');
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
			xlabel("Time (seconds)");
			set(gca(),'ycolor','b');
			hold off;
		end
	end
	%process and plot the IMU data, use three subplots, for accel, gyro, and magno
	figure();
	clf;
	leg={'x','y','z'};
	subplot(3,1,1);
	imut=[0:length(imud)-1]./sprimu;
	plot(imut,imud(:,1:3).*2000);
	legend(leg);
	set(gca(),'ycolor','b');
	ylabel("Rotation rate (^{o}s^{-1})");
	xlim([min(imut) max(imut)]);
	subplot(3,1,2);
	plot(imut,imud(:,4:6).*8);
	ylabel("Acceleration (G)");
	set(gca(),'ycolor','b');
	xlim([min(imut) max(imut)]);
	subplot(3,1,3);
	imud(find(abs(imud(:,10).*2^15)>90),10)=NA;
	temp=imud(:,10).*2^15;
	ind=find(~isna(temp));
	temp=interp1(ind,temp(ind),[1:length(temp)]);
	[a,b]=butter(2,0.01);
	temp=filtfilt(a,b,temp);
	imud(:,7:8).*=-1;
	[ax,h1,h2]=plotyy(imut,imud(:,7:9).*4,imut,temp);
	ylabel(ax(1),"Magnetic flux (Gs)");
	ylabel(ax(2),"Temperature ^{o}C");
	xlabel("Time (seconds)");
	set(h2,'color','k');
	set(ax(2),'ycolor','k');
	set(ax(1),'xlim', [min(imut) max(imut)]);
	set(ax(2),'xlim', [min(imut) max(imut)]);
	n=0;
endfunction
