#include <math.h>
#include <string.h>
#include "dcm_attitude.h"

float PI_glob[3][2];
float I_limits[3];

float DCM_glob[3][3]={{1,0,0}, {0,1,0}, {0,0,1}};//Initialised as world ==  body

//TODO check it!
//z axes need reversing on all three sensors, also swap x and y on the magno 

//Main filter code, runs with corrected accel and magno (gain and offset). Gyro should also be scaled to radian per second units, others can be unscaled
//Returns the magnitude of the acceleration vector
float main_filter(float DCM[3][3], float magno[3], float accel[3], float euler_out[3], float gyro[3], float d_t) {
	static float I[3];//Integral offset
	float magnitude;
	//First set is to run the correction filter
	float corr[3],corrections[3];
	vector_by_matrix(corr, magno, DCM);//DCM is body to world
	corrections[2]=magno_correction(corr);
	vector_by_matrix(corr, accel, DCM);
	magnitude=accel_correction(corrections, corr);//The corrections three vector is now a set of roll, pitch, and yaw corrections
	vector_by_matrix_transpose(corr, corrections, DCM);//Move correction back to the body frame using the transpose
	run_3_pi(corrections, I, PI_glob, I_limits, corr, d_t);//The PI filtering on the gyros takes place in the sensor body frame
	correct_gyro(corrections, gyro, gyro);//Correct the gyro in place
	//Now propogate
	propogate_gyro(DCM, gyro, d_t);
	normalize_DCM(DCM);
	DCM_to_euler(euler_out, DCM);
	return magnitude;
}

//Used to init the limits on I and PI values, assumes they are all the same
void init_controller(float PI_limit[3]) {
	for(uint8_t n=0; n<3; n++) {
		PI_glob[n][0]=PI_limit[0];
		PI_glob[n][1]=PI_limit[1];
		I_limits[n]=PI_limit[2];
	}		
}

//Simple addition
void correct_gyro(float correction[3], float gyro[3], float out[3]) {
	for(uint8_t n=0; n<3; n++)
		out[n]=gyro[n]+correction[n];
}

//Runs three PI filters to correct the current attitude by manipulating the gyor signals
void run_3_pi(float out[3], float I[3], float PI[3][2], float I_limit[3], float error[3], float d_t) {
	for(uint8_t n=0; n<3; n++) {
		I[n]+=d_t*error[n];
		if(fabs(I[n])>I_limit[n])//enforce range limit on I
			I[n]=I[n]>0?I_limit[n]:-I_limit[n];
		out[n]=PI[n][0]*error[n]+PI[n][1]*I[n];
	}
}

//Cross product the input accel, magnitude corrects the input pointer. Output is pitch and roll correction in radians
float accel_correction(float out[2], float accel[3]) {
	const float reference[3]={0,0,-1};//NED space
	float vect[3],vect_[3],magnitude;//Temp usage
	magnitude=normalize_vector(vect, accel);//vect is the output
	cross_product(vect_,vect,reference);
	out[0]=vect_[0];
	out[1]=vect_[1];//The x,y components of the cross product, for small angles this is the rotation in radians
	return magnitude;
}

//Cross product the input magno, magnitude corrects the input pointer. Output is yaw correction in radians
float magno_correction(float magno[3]) {
	const float reference[3]={1,0,0};//NED space
	float vect[3],vect_[3];//Temp usage
	normalize_vector(vect, magno);//vect is the output
	vect[2]=0;	//The z axis of the corrected magno is zeroed
	cross_product(vect_,vect,reference);
	return vect_[2];//The z component of the cross product, for small angles this is the rotation in radians
}

//Cross product of two input vectors, used for comparing accel to gravity
void cross_product(float out[3], float in_one[3], float in_two[3]) {
	out[0]=in_one[1]*in_two[2]-in_one[2]*in_two[1];
	out[1]=in_one[0]*in_two[2]-in_one[2]*in_two[0];
	out[2]=in_one[0]*in_two[1]-in_one[1]*in_two[0];
}

//Normalizes a three vector, returns its original length as a float
float normalize_vector(float out[3], float in[3]) {
	float abs=sqrtf(in[0]*in[0]+in[1]*in[1]+in[2]*in[2]);
	for(uint8_t n=0; n<3; n++)
		out[n]=in[n]/abs;
	return abs;
}

//Multiplies a 3 vector by a 3x3 matrix, useful for translating sensor vectors by the DCM, to move them from body space to world space
void vector_by_matrix(float out[3], float in[3], float matrix[3][3]) {
	for(uint8_t n=0; n<3; n++)
		out[n]=matrix[n][0]*in[0]+matrix[n][1]*in[1]+matrix[n][2]*in[2];
}

//Multiples a 3 vector by the transpose of a 3x3 matrix (used for reverse rotation operations)
void vector_by_matrix_transpose(float out[3], float in[3], float matrix[3][3]) {
	for(uint8_t n=0; n<3; n++)
		out[n]=matrix[0][n]*in[0]+matrix[1][n]*in[1]+matrix[2][n]*in[2];//Reverse row and column indices
}

//Normalizes the DCM, passed pointer to 3x3 DCM matrix of floats
void normalize_DCM(float DCM[3][3]) {
	float error=DCM[0][0]*DCM[1][0]+DCM[0][1]*DCM[1][1]+DCM[0][2]*DCM[1][2];//dot product of the first two rows
	float DCM_[3][3];
	memcpy(DCM_,DCM,9*sizeof(float));
	DCM[0][0]-=(error/2)*DCM_[1][0];//X-=(error/2).*Y;
	DCM[0][1]-=(error/2)*DCM_[1][1];//Note that we use the copied version to correct here
	DCM[0][2]-=(error/2)*DCM_[1][2];
	DCM[1][0]-=(error/2)*DCM_[0][0];//Y-=(error/2).*X;
	DCM[1][1]-=(error/2)*DCM_[0][1];
	DCM[1][2]-=(error/2)*DCM_[0][2];
	DCM[2][0]=DCM[0][1]*DCM[1][2]-DCM[0][2]*DCM[1][1];//Z=X x Y;
	DCM[2][1]=DCM[0][0]*DCM[1][2]-DCM[0][2]*DCM[1][0];
	DCM[2][2]=DCM[0][0]*DCM[1][1]-DCM[0][1]*DCM[1][0];
	for(uint8_t n=0; n<3; n++) {
		error=DCM[n][0]*DCM[n][0]+DCM[n][1]*DCM[n][1]+DCM[n][2]*DCM[n][2];//Dot product correct each row (dot product of itself)
		error=(3-error)/2;
		DCM[n][0]*=error;
		DCM[n][1]*=error;
		DCM[n][2]*=error;
	}
}

//Update the DCM using the current gyro data
void propogate_gyro(float DCM[3][3], float gyro[3], float delta_t) {
	for(uint8_t n=0; n<3; n++) {//Go through each row and multiply
		DCM[n][0]+=delta_t*(gyro[2]*DCM[n][1]-gyro[1]*DCM[n][2]);
		DCM[n][1]+=delta_t*(gyro[0]*DCM[n][2]-gyro[2]*DCM[n][0]);
		DCM[n][2]+=delta_t*(gyro[1]*DCM[n][0]-gyro[0]*DCM[n][1]);
	}
}

//Convert a rotation matrix to a quaternion
void DCM_to_quaternion(float quat[4], float DCM[3][3]) {
	quat[0]=(1/2)*sqrtf(1+DCM[0][0]+DCM[1][1]+DCM[2][2]);//Diagonal
	quat[1]=(1/(4*quat[0]))*(DCM[2][1]-DCM[1][2]);
	quat[2]=(1/(4*quat[0]))*(DCM[0][2]-DCM[2][0]);
	quat[3]=(1/(4*quat[0]))*(DCM[1][0]-DCM[0][1]);
}

//Convert a rotation matrix to euler angles
void DCM_to_euler(float euler[3], float DCM[3][3]) {
	euler[0]=atan2f(DCM[2][0],DCM[2][1]);
	euler[1]=acosf(DCM[2][2]);
	euler[2]=-atan2f(DCM[0][2],DCM[1][2]);
}
