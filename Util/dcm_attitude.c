





//Normalizes the DCM, passed pointer to 3x3 DCM matrix of floats
void normalize_DCM(float DCM[3][3]) {
	float error=DCM[0][0]*DCM[1][0]+DCM[0][1]*DCM[1][1]+DCM[0][2]*DCM[1][2];//dot product of the first two rows
	DCM[0][0]-=(error/2)*DCM[1][0];//X-=(error/2).*Y;
	DCM[0][1]-=(error/2)*DCM[1][1];
	DCM[0][2]-=(error/2)*DCM[1][2];
	DCM[1][0]-=(error/2)*DCM[0][0];//Y-=(error/2).*X;
	DCM[1][1]-=(error/2)*DCM[0][1];
	DCM[1][2]-=(error/2)*DCM[0][2];
	DCM[2][0]=DCM[0][1]*DCM[1][2]-DCM[0][2]*DCM[1][1];//Z=X x Y;
	DCM[2][0]=DCM[0][0]*DCM[1][2]-DCM[0][2]*DCM[1][0];
	DCM[3][1]=DCM[0][0]*DCM[1][1]-DCM[0][1]*DCM[1][0];
	for(uint8_t n=0; n<3; n++) {
		float error=DCM[n][0]*DCM[n][0]+DCM[n][1]*DCM[n][1]+DCM[n][2]*DCM[n][2];//Dot product correct each row (dot product of itself)
		error=(3-error)/2;
		DCM[n][0]*=error;
		DCM[n][1]*=error;
		DCM[n][2]*=error;
	}
}

//Update the DCM using the current gyro data
void propogate_gyro(float DCM[3][3], float gyro[3], float delta_t) {
	DCM[0][1]-=delta_t*gyro[2];
	DCM[0][2]+=delta_t*gyro[1];
	DCM[1][0]+=delta_t*gyro[2];
	DCM[1][2]-=delta_t*gyro[0];
	DCM[2][0]-=delta_t*gyro[1];
	DCM[2][1]+=delta_t*gyro[0];
}
