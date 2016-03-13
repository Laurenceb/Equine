function dataout=sim_filter(data)
	z_one=[0,0];
	z_two=[0,0];
	for n=1:length(data)
		in=data(n)+1.322*z_one(1)-0.339*z_one(2);
		out=0.331*in-0.331*z_one(2);
		z_one(2)=z_one(1);
		z_one(1)=in;
		in=out+0.46*z_two(1)-0.486*z_two(2);
		dataout(n)=0.743*in-0.46*z_two(1)+0.743*z_two(2);
		z_two(2)=z_two(1);
		z_two(1)=in;
	end
endfunction
