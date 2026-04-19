void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,float* pitch,float* roll,float* yaw) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;
	
	// normalise the measurements
	norm = sqrt(ax*ax + ay*ay + az*az);   
	if (norm >= 0.01)
	{	
		ax = ax / norm;
		ay = ay / norm;
		az = az / norm;  
	}	
	else
	{
		ax=0;
		ay=0;
		az=0;
	}
	
	// estimated direction of gravity
	vx = 2*(q1*q3 - q0*q2);
	vy = 2*(q0*q1 + q2*q3);
	vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	// error is sum of cross product between reference direction of field and direction measured by sensor
	ex = (ay*vz - az*vy);
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	// integral error scaled integral gain
	exInt = exInt + ex*Ki*2.0*halfT;
	eyInt = eyInt + ey*Ki*2.0*halfT;
	ezInt = ezInt + ez*Ki*2.0*halfT;
	
	// adjusted gyroscope measurements
	gx = gx + Kp*ex + exInt;
	gy = gy + Kp*ey + eyInt;
	gz = gz + Kp*ez + ezInt;
	
	// integrate quaternion rate and normalise
	q0 = q0_last + (-q1_last*gx - q2_last*gy - q3_last*gz)*halfT;
	q1 = q1_last + (q0_last*gx + q2_last*gz - q3_last*gy)*halfT;
	q2 = q2_last + (q0_last*gy - q1_last*gz + q3_last*gx)*halfT;
	q3 = q3_last + (q0_last*gz + q1_last*gy - q2_last*gx)*halfT;  
	
	// normalise quaternion
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if (norm >= 0.01)
	{	
		q0 = q0 / norm;
		q1 = q1 / norm;
		q2 = q2 / norm;
		q3 = q3 / norm; 
	}	
	else
	{
		q0 = 0;
		q1 = 0;
		q2 = 0;
		q3 = 0;
	}
	
	//计算得到俯仰角/横滚角/航向角
	*pitch = asin(-2*q1*q3 + 2*q0*q2)*57.3;	
	*roll  = atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)*57.3;
	*yaw = atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)*57.3;
	
	q0_last = q0;
	q1_last = q1;
	q2_last = q2;
	q3_last = q3;
		
}