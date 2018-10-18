#ifndef KALMAN3_H_
#define KALMAN3_H_

#include <opencv2/core/core.hpp>

using namespace std;

#define EARTH_GRAVITY   9.81

class Kalman2
{
public:
	//POSITION STATE
	cv::Mat KalmanP2,KalmanQ2,KalmanR2,KalmanR3;
	
	cv::Mat H_GPS,H_cam;
	
	double State2[16];
			
	bool update2;

	cv::Mat qtLastUpdate;
	
	cv::Mat tfSolution2;
	cv::Mat qtSolution2;

	double m_dPositionUpdateR;
	double m_dAccelPredictQ;
	double m_dGyroPredictQ;
	
	double dt,dt2;	// time step

  Kalman2()
  {
  	update2=true;

		//dt=0.02;		// 50 Hz
		//dt=0.01;		// 100 Hz
		dt=0.005;		// 200 Hz
		//dt=0.0025;	// 400 Hz

		dt2=dt*dt;
		
		for (int i=0;i<16;i++) State2[i]=0;
  	State2[3]=1;

		KalmanP2=cv::Mat::eye(16, 16, CV_64F);
		
		// UNCERTAINTY OF PREDICTION MODEL ( w,w_bias,a,a_bias)
		KalmanQ2=cv::Mat::eye(12, 12, CV_64F);
		
		KalmanR2=cv::Mat::eye(3, 3, CV_64F);
		
		KalmanR3=cv::Mat::eye(7, 7, CV_64F);
		
		// measurement model partial derivative matrix
		H_GPS=cv::Mat::zeros(3,16,CV_64F);// 3 measurements (xyz), 16 states
		// position in global frame is measured directly (state 7-9)
		H_GPS.at<double>(0,7)=1;
		H_GPS.at<double>(1,8)=1;
		H_GPS.at<double>(2,9)=1;		

		
		tfSolution2=cv::Mat::eye(4, 4, CV_64F);
		qtSolution2=cv::Mat::zeros(4, 1, CV_64F);qtSolution2.at<double>(3)=1.0;
			
		// WARNING - IF Q IS GREATER THAN SOME THRESHOLD, P-MATRIX WILL DIVERGE !!!
		// ALWAYS SHOW P-MATRIX VALUES WHILE TUNING THE FILTER !!!

		setGyroPredictQ(0.0001);

		KalmanQ2.at<double>(3,3)=0.0000001f;			// UNCERTAINITY IN PREDICTION OF GYRO BIAS ( BIAS RANDOM WALK )
		KalmanQ2.at<double>(4,4)=0.0000001f;
		KalmanQ2.at<double>(5,5)=0.0000001f;

		setAccelPredictQ(0.1);

		KalmanQ2.at<double>(9,9)		=0.0001f;	// UNCERTAINITY IN PREDICTION OF ACCEL BIAS
		KalmanQ2.at<double>(10,10)	=0.0001f;
		KalmanQ2.at<double>(11,11)	=0.0001f;
		
		setPositionUpdateR(0.0001);	

		// NOISE OF QUATERNION+POSITION MEASUREMENT	
		KalmanR3.at<double>(0,0)=0.005f;
		KalmanR3.at<double>(1,1)=0.005f;
		KalmanR3.at<double>(2,2)=0.005f;
		
		KalmanR3.at<double>(3,3)=0.0001f;
		KalmanR3.at<double>(4,4)=0.0001f;
		KalmanR3.at<double>(5,5)=0.0001f;
		KalmanR3.at<double>(6,6)=0.0001f;
			
  }  
  ~Kalman2()
  {}

	void setRate(int freq)
	{
		dt=1/(double)freq;
		dt2=dt*dt;
	}

	void setPositionUpdateR(double var)
	{
		m_dPositionUpdateR=var;							// store in single variable for display
		KalmanR2.at<double>(0,0)=var;				// UNCERTAINITY OF POSITION MEASUREMENTS
		KalmanR2.at<double>(1,1)=var;
		KalmanR2.at<double>(2,2)=var;
	}

	void setAccelPredictQ(double var)
	{
		m_dAccelPredictQ=var;								// store in single variable for display
		KalmanQ2.at<double>(6,6)=var;				// UNCERTAINITY OF ACCELEROMETER MEASUREMENTS
		KalmanQ2.at<double>(7,7)=var;
		KalmanQ2.at<double>(8,8)=var;
	}

	void setGyroPredictQ(double var)
	{
		m_dGyroPredictQ=var;								// store in single variable for display
		KalmanQ2.at<double>(0,0)=var;				// UNCERTAINITY OF GYROSCOPE MEASUREMENT w
		KalmanQ2.at<double>(1,1)=var;
		KalmanQ2.at<double>(2,2)=var;
	}

	void setEuler(cv::Mat &quaternion, const double& yaw, const double& pitch, const double& roll)
	{
		double halfYaw = yaw * 0.5;  
		double halfPitch = pitch * 0.5;  
		double halfRoll = roll * 0.5;  
		double cosYaw = cos(halfYaw);
		double sinYaw = sin(halfYaw);
		double cosPitch = cos(halfPitch);
		double sinPitch = sin(halfPitch);
		double cosRoll = cos(halfRoll);
		double sinRoll = sin(halfRoll);
		quaternion.at<double>(0)=cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
		quaternion.at<double>(1)=cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
		quaternion.at<double>(2)=sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
		quaternion.at<double>(3)=cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	}

  void setEulerYPR(cv::Mat &rotMat,double eulerZ, double eulerY,double eulerX)  
	{ 
		double ci = cos(eulerX); 
		double cj = cos(eulerY); 
		double ch = cos(eulerZ); 
		double si = sin(eulerX); 
		double sj = sin(eulerY); 
		double sh = sin(eulerZ); 
		double cc = ci * ch; 
		double cs = ci * sh; 
		double sc = si * ch; 
		double ss = si * sh;

		rotMat.at<double>(0,0)= cj * ch;	rotMat.at<double>(0,1)= sj * sc - cs;		rotMat.at<double>(0,2)= sj * cc + ss,
		rotMat.at<double>(1,0)= cj * sh;	rotMat.at<double>(1,1)= sj * ss + cc;		rotMat.at<double>(1,2)= sj * cs - sc, 
		rotMat.at<double>(2,0)=-sj;				rotMat.at<double>(2,1)= cj * si;				rotMat.at<double>(2,2)= cj * ci;
	}

	void getEulerYPR(double& yaw, double& pitch, double& roll, const cv::Mat &rotMat, unsigned int solution_number = 1) const
	{
		#define PI 3.1415926535897932384626433832795
		struct Euler
		{
			double yaw;
			double pitch;
			double roll;
		};

    Euler euler_out;
    Euler euler_out2; //second solution
    //get the pointer to the raw data
 
		// Check that pitch is not at a singularity
		// Check that pitch is not at a singularity
    if (fabs(rotMat.at<double>(2,0)) >= 1)
    {
			euler_out.yaw = 0;
			euler_out2.yaw = 0;
         
			// From difference of angles formula
			if (rotMat.at<double>(2,0) < 0)  //gimbal locked down
			{
				double delta = atan2(rotMat.at<double>(0,1),rotMat.at<double>(0,2));
							euler_out.pitch = PI / 2.0;
							euler_out2.pitch = PI / 2.0;
							euler_out.roll = delta;
							euler_out2.roll = delta;
			}
			else // gimbal locked up
			{
				double delta = atan2(-rotMat.at<double>(0,1),-rotMat.at<double>(0,2));
							euler_out.pitch = -PI / 2.0;
							euler_out2.pitch = -PI / 2.0;
							euler_out.roll = delta;
							euler_out2.roll = delta;
			}
    }
    else
    {
			euler_out.pitch = - asin(rotMat.at<double>(2,0));
			euler_out2.pitch = PI - euler_out.pitch;
 
			euler_out.roll = atan2(rotMat.at<double>(2,1)/cos(euler_out.pitch), rotMat.at<double>(2,2)/cos(euler_out.pitch));
			euler_out2.roll= atan2(rotMat.at<double>(2,1)/cos(euler_out2.pitch),rotMat.at<double>(2,2)/cos(euler_out2.pitch));
 
			euler_out.yaw = atan2(rotMat.at<double>(1,0)/cos(euler_out.pitch), rotMat.at<double>(0,0)/cos(euler_out.pitch));
			euler_out2.yaw= atan2(rotMat.at<double>(1,0)/cos(euler_out2.pitch),rotMat.at<double>(0,0)/cos(euler_out2.pitch));
    }
 
    if (solution_number == 1)
    { 
			yaw = euler_out.yaw; 
			pitch = euler_out.pitch;
			roll = euler_out.roll;
    }
    else
    { 
			yaw = euler_out2.yaw; 
			pitch = euler_out2.pitch;
			roll = euler_out2.roll;
    }
  }

	void mat2quat(cv::Mat &q, cv::Mat &rotMat)
	{
		double trace = rotMat.at<double>(0,0) + rotMat.at<double>(1,1) + rotMat.at<double>(2,2);
		double temp[4];
	
		if (trace > 0.0) 
		{
			double s = sqrt(trace + 1.0);
			temp[3]=(s * 0.5);
			s = 0.5 / s;

			temp[0]=((rotMat.at<double>(2,1) - rotMat.at<double>(1,2)) * s);
			temp[1]=((rotMat.at<double>(0,2) - rotMat.at<double>(2,0)) * s);
			temp[2]=((rotMat.at<double>(1,0) - rotMat.at<double>(0,1)) * s);
		} 
		else 
		{
			int i = rotMat.at<double>(0,0) < rotMat.at<double>(1,1) ? (rotMat.at<double>(1,1) < rotMat.at<double>(2,2) ? 2 : 1) :
				(rotMat.at<double>(0,0) < rotMat.at<double>(2,2) ? 2 : 0); 
			int j = (i + 1) % 3;  
			int k = (i + 2) % 3;

			double s = sqrt(rotMat.at<double>(i,i) - rotMat.at<double>(j,j) - rotMat.at<double>(k,k) + 1.0);
			temp[i] = s * 0.5;
			s = 0.5 / s;

			temp[3] = (rotMat.at<double>(k,j) - rotMat.at<double>(j,k)) * s;
			temp[j] = (rotMat.at<double>(j,i) + rotMat.at<double>(i,j)) * s;
			temp[k] = (rotMat.at<double>(k,i) + rotMat.at<double>(i,k)) * s;
		}
		q.at<double>(0)=temp[0];
		q.at<double>(1)=temp[1];
		q.at<double>(2)=temp[2];
		q.at<double>(3)=temp[3];
	}

	void quat2mat(cv::Mat &rotMat, const cv::Mat &q) 
	{
		double d = sqrt(q.at<double>(0)*q.at<double>(0) + q.at<double>(1)*q.at<double>(1)
			+ q.at<double>(2)*q.at<double>(2) + q.at<double>(3)*q.at<double>(3));

		assert(d != 0.0);
		double s = 2.0 / d;
		double xs = q.at<double>(0) * s,   ys = q.at<double>(1) * s,   zs = q.at<double>(2) * s;
		double wx = q.at<double>(3) * xs,  wy = q.at<double>(3) * ys,  wz = q.at<double>(3) * zs;
		double xx = q.at<double>(0) * xs,  xy = q.at<double>(0) * ys,  xz = q.at<double>(0) * zs;
		double yy = q.at<double>(1) * ys,  yz = q.at<double>(1) * zs,  zz = q.at<double>(2) * zs;
		rotMat.at<double>(0,0)=1.0 - (yy + zz);	rotMat.at<double>(0,1) = xy - wz;					rotMat.at<double>(0,2) = xz + wy;
		rotMat.at<double>(1,0)=xy + wz;					rotMat.at<double>(1,1) = 1.0 - (xx + zz); rotMat.at<double>(1,2) = yz - wx;
		rotMat.at<double>(2,0)=xz - wy;					rotMat.at<double>(2,1) = yz + wx;					rotMat.at<double>(2,2) = 1.0 - (xx + yy);
	}


	void InitState(double ax, double ay, double az, double gyroX, double gyroY, double gyroZ, double positionX, double positionY, double positionZ)
	{		
		double g=-sqrt(ax*ax+ay*ay+az*az);
		 
		double roll=atan2(-ay/g,-az/g);			//ROLL (around X-axis) - 
		double pitch=-asin(-ax/g);					//PITCH (around Y-axis - heading)
		double yaw=0;												//YAW (around Z axis) - BANKING
				
		//cout<<"STATE INIT: YPR= "<<yaw*180/3.14<<" 0 "<<roll*180/3.14<<endl;
				
		cv::Mat mat=cv::Mat::eye(3, 3, CV_64F);;
		setEulerYPR(mat,yaw,pitch,roll);		
		mat2quat(qtSolution2,mat);
		qtLastUpdate=qtSolution2;
		
		State2[0]=qtSolution2.at<double>(0);
		State2[1]=qtSolution2.at<double>(1);
		State2[2]=qtSolution2.at<double>(2);
		State2[3]=qtSolution2.at<double>(3);
		
		// gyro bias
		State2[4]=0;
		State2[5]=0;
		State2[6]=0;

		// position
		State2[7]=positionX;
		State2[8]=positionY;
		State2[9]=positionZ;

		// velocity
		State2[10]=0;
		State2[11]=0;
		State2[12]=0;
				
		//accel bias
		State2[13]=0;
		State2[14]=0;
		State2[15]=0;
			
	}

	void getPosition(double &x, double &y, double &z) {
		x = -State2[8];
		y = -State2[9];
		z = State2[7];
	}

	void getQ(double &x, double &y, double &z, double &w) {
		x = State2[0];
		y = State2[1];
		z = State2[2];
		w = State2[3];
	}

	void KalmanPredict2(double wx, double wy, double wz, double ax, double ay, double az)
	{
				
		double qx=State2[0];
		double qy=State2[1];
		double qz=State2[2];
		double qw=State2[3];

		//GYRO BIASES
		double bx=State2[4];
		double by=State2[5];
		double bz=State2[6];

		//POSITION IN WORLD FRAME...
		double px=State2[7];
		double py=State2[8];
		double pz=State2[9];

		//VELOCITY IN WORLD FRAME...
		double vx=State2[10];
		double vy=State2[11];
		double vz=State2[12];
		
		//ACCELERATION BIASES
		double abx=State2[13];
		double aby=State2[14];
		double abz=State2[15];
				
		// partial derivative of the new state / state variables 		
		cv::Mat F=cv::Mat::eye(16, 16, CV_64F);
		
		//PREDICTION OF THE NEW STATE	
		State2[0]=qx+0.5*dt*(+qw*(wx-bx)-qz*(wy-by)+qy*(wz-bz));			
		State2[1]=qy+0.5*dt*(+qz*(wx-bx)+qw*(wy-by)-qx*(wz-bz));		
		State2[2]=qz+0.5*dt*(-qy*(wx-bx)+qx*(wy-by)+qw*(wz-bz));
		State2[3]=qw+0.5*dt*(-qx*(wx-bx)-qy*(wy-by)-qz*(wz-bz));
		
		//NORMALIZE QUATERNION...
		double length=sqrt(State2[0]*State2[0]+State2[1]*State2[1]+State2[2]*State2[2]+State2[3]*State2[3]);
		State2[0]/=length;	State2[1]/=length;	State2[2]/=length;	State2[3]/=length;				

		
		// partial derivative of the new qx	************************************************************************************************	
		//d /dqx													//d /dqy													//d /dqz													//d /dqw
		F.at<double>(0,0)= 1;							F.at<double>(0,1)=0.5*dt*(wz-bz);	F.at<double>(0,2)=-0.5*dt*(wy-by);F.at<double>(0,3)=0.5*dt*(wx-bx);
		//d /dbx													//d /dby													//d /dbz	
		F.at<double>(0,4)=-0.5*dt*qw;			F.at<double>(0,5)=+0.5*dt*qz;			F.at<double>(0,6)=-0.5*dt*qy;
		//d /dpx													//d /dpy													//d /dpz	
		F.at<double>(0,7)=0;							F.at<double>(0,8)=0;							F.at<double>(0,9)=0;
		//d /dvx													//d /dvy													//d /dvz	
		F.at<double>(0,10)=0;							F.at<double>(0,11)=0;							F.at<double>(0,12)=0;
		//d /dabx													//d /daby													//d /dabz	
		F.at<double>(0,13)=0;							F.at<double>(0,14)=0;							F.at<double>(0,15)=0;
		
		// partial derivatives of the new qy *********************************************************************************************
		F.at<double>(1,0)=-0.5*dt*(wz-bz);F.at<double>(1,1)= 1.0;						F.at<double>(1,2)=+0.5*dt*(wx-bx);F.at<double>(1,3)=0.5*dt*(wy-by);
		F.at<double>(1,4)=-0.5*dt*qz;			F.at<double>(1,5)=-0.5*dt*qw;			F.at<double>(1,6)=+0.5*dt*qx;		
		F.at<double>(1,7)=0;							F.at<double>(1,8)=0;							F.at<double>(1,9)=0;
		F.at<double>(1,10)=0;							F.at<double>(1,11)=0;							F.at<double>(1,12)=0;
		F.at<double>(1,13)=0;							F.at<double>(1,14)=0;							F.at<double>(1,15)=0;
		
			
		// partial derivatives of the new qz *********************************************************************************************
		F.at<double>(2,0)=+0.5*dt*(wy-by);F.at<double>(2,1)=-0.5*dt*(wx-bx);F.at<double>(2,2)= 1.0;						F.at<double>(2,3)=0.5*dt*(wz-bz);
		F.at<double>(2,4)=+0.5*dt*qy;			F.at<double>(2,5)=-0.5*dt*qx;			F.at<double>(2,6)=-0.5*dt*qw;
		F.at<double>(2,7)=0;							F.at<double>(2,8)=0;							F.at<double>(2,9)=0;
		F.at<double>(2,10)=0;							F.at<double>(2,11)=0;							F.at<double>(2,12)=0;
		F.at<double>(2,13)=0;							F.at<double>(2,14)=0;							F.at<double>(2,15)=0;

		// partial derivatives of the new qw *********************************************************************************************
		F.at<double>(3,0)=-0.5*dt*(wx-bx);F.at<double>(3,1)=-0.5*dt*(wy-by);F.at<double>(3,2)=-0.5*dt*(wz-bz);F.at<double>(3,3)=1;								
		F.at<double>(3,4)= 0.5*dt*qx;			F.at<double>(3,5)= 0.5*dt*qy;			F.at<double>(3,6)= 0.5*dt*qz;
		F.at<double>(3,7)=0;							F.at<double>(3,8)=0;							F.at<double>(3,9)=0;
		F.at<double>(3,10)=0;							F.at<double>(3,11)=0;							F.at<double>(3,12)=0;
		F.at<double>(3,13)=0;							F.at<double>(3,14)=0;							F.at<double>(3,15)=0;

		// PREDICTION OF THE NEW GYRO BIAS - State2[4-6] REMAINS THE SAME ...
		
		// partial derivatives of the new bx *********************************************************************************************
		F.at<double>(4,0)=0;							F.at<double>(4,1)=0;							F.at<double>(4,2)=0;							F.at<double>(4,3)=0;
		F.at<double>(4,4)=1;							F.at<double>(4,5)=0;							F.at<double>(4,6)=0;
		F.at<double>(4,7)=0;							F.at<double>(4,8)=0;							F.at<double>(4,9)=0;
		F.at<double>(4,10)=0;							F.at<double>(4,11)=0;							F.at<double>(4,12)=0;		
		F.at<double>(4,13)=0;							F.at<double>(4,14)=0;							F.at<double>(4,15)=0;
		
		// partial derivatives of the new by *********************************************************************************************
		F.at<double>(5,0)=0;							F.at<double>(5,1)=0;							F.at<double>(5,2)=0;							F.at<double>(5,3)=0;
		F.at<double>(5,4)=0;							F.at<double>(5,5)=1;							F.at<double>(5,6)=0;
		F.at<double>(5,7)=0;							F.at<double>(5,8)=0;							F.at<double>(5,9)=0;
		F.at<double>(5,10)=0;							F.at<double>(5,11)=0;							F.at<double>(5,12)=0;
		F.at<double>(5,13)=0;							F.at<double>(5,14)=0;							F.at<double>(5,15)=0;
		
		// partial derivatives of the new bz *********************************************************************************************
		F.at<double>(6,0)=0;							F.at<double>(6,1)=0;							F.at<double>(6,2)=0;							F.at<double>(6,3)=0;
		F.at<double>(6,4)=0;							F.at<double>(6,5)=0;							F.at<double>(6,6)=1;
		F.at<double>(6,7)=0;							F.at<double>(6,8)=0;							F.at<double>(6,9)=0;
		F.at<double>(6,10)=0;							F.at<double>(6,11)=0;							F.at<double>(6,12)=0;
		F.at<double>(6,13)=0;							F.at<double>(6,14)=0;							F.at<double>(6,15)=0;

		// PREDICTION OF THE NEW POSITION IN THE WORLD FRAME
		
		// FIRST CALCULATE ACCELERATIONS IN THE WORLD FRAME...
		double awx=	(1-2*qy*qy-2*qz*qz)*(ax-abx)		+2*(qx*qy-qz*qw)*(ay-aby)			+2*(qx*qz+qy*qw)*(az-abz);	
		double awy=			2*(qx*qy+qz*qw)*(ax-abx)+(1-2*qx*qx-2*qz*qz)*(ay-aby)			+2*(qy*qz-qx*qw)*(az-abz);		
		double awz=			2*(qx*qz-qy*qw)*(ax-abx)		+2*(qy*qz+qx*qw)*(ay-aby)	+(1-2*qx*qx-2*qy*qy)*(az-abz);
		
		// ...THEN PREDICT NEW POSITION IN THE WORLD FRAME...
		State2[7]=px+vx*dt+(awx)*dt2*0.5;
		State2[8]=py+vy*dt+(awy)*dt2*0.5;
		State2[9]=pz+vz*dt+(awz-EARTH_GRAVITY)*dt2*0.5;

		// partial derivatives of the new px *********************************************************************************************		
		// d /d qx	ok																									// d / d qy ok																				
		F.at<double>(7,0)=dt2*(qy*(ay-aby)+qz*(az-abz));								F.at<double>(7,1)=dt2*(-2*qy*(ax-abx)+qx*(ay-aby)+qw*(az-abz));
		// d /d qz	ok																									// d / d qw ok
		F.at<double>(7,2)=dt2*(-2*qz*(ax-abx)-qw*(ay-aby)+qx*(az-abz));	F.at<double>(7,3)=dt2*(-qz*(ay-aby)+qy*(az-abz));
		// d /d bx																	// d /d by																	// d /d bz
		F.at<double>(7,4)=0;												F.at<double>(7,5)=0;												F.at<double>(7,6)=0;
		// d /d px																	// d /d py																	// d /d pz
		F.at<double>(7,7)=1;												F.at<double>(7,8)=0;												F.at<double>(7,9)=0;
		// d /d vx																	// d /d vy																	// d /d vz
		F.at<double>(7,10)=dt;											F.at<double>(7,11)=0;												F.at<double>(7,12)=0;
		// d /d abx																	// d / d aby																// d /d abz
		F.at<double>(7,13)=-dt2*(0.5-qy*qy-qz*qz);	F.at<double>(7,14)=-dt2*(qx*qy-qz*qw);			F.at<double>(7,15)=-dt2*(qx*qz+qy*qw);
			
		// partial derivatives of the new py *********************************************************************************************		
		// d /d qx	ok																									// d /d qy ok																				
		F.at<double>(8,0)=dt2*(qy*(ax-abx)-2*qx*(ay-aby)-qw*(az-abz));	F.at<double>(8,1)=dt2*(qx*(ax-abx)+qz*(az-abz));
		// d /d qz	ok																									// d /d qw ok
		F.at<double>(8,2)=dt2*(qw*(ax-abx)-2*qz*(ay-aby)+qy*(az-abz));	F.at<double>(8,3)=dt2*(qz*(ax-abx)-qx*(az-abz));
		F.at<double>(8,4)=0;												F.at<double>(8,5)=0;												F.at<double>(8,6)=0;
		F.at<double>(8,7)=0;												F.at<double>(8,8)=1;												F.at<double>(8,9)=0;
		F.at<double>(8,10)=0;												F.at<double>(8,11)=dt;											F.at<double>(8,12)=0;
		F.at<double>(8,13)=-dt2*(qx*qy+qz*qw);			F.at<double>(8,14)=-dt2*(0.5-qx*qx-qz*qz);	F.at<double>(8,15)=-dt2*(qy*qz-qx*qw);

		// partial derivatives of the new pz *********************************************************************************************		
		// d /d qx ok																										// d /d qy ok																				
		F.at<double>(9,0)=dt2*(qz*(ax-abx)+qw*(ay-aby)-2*qx*(az-abz));	F.at<double>(9,1)=dt2*(-qw*(ax-abx)+qz*(ay-aby)-2*qy*(az-abz));
		// d /d qz ok																										// d /d qw ok
		F.at<double>(9,2)=dt2*(qx*(ax-abx)+qy*(ay-aby));								F.at<double>(9,3)=dt2*0.5*2*(-qy*(ax-abx)+qx*(ay-aby));
		F.at<double>(9,4)=0;												F.at<double>(9,5)=0;												F.at<double>(9,6)=0;
		F.at<double>(9,7)=0;												F.at<double>(9,8)=0;												F.at<double>(9,9)=1;
		F.at<double>(9,10)=0;												F.at<double>(9,11)=0;												F.at<double>(9,12)=dt;
		F.at<double>(9,13)=-dt2*(qx*qz-qy*qw);			F.at<double>(9,14)=-dt2*(qy*qz+qx*qw);			F.at<double>(9,15)=-dt2*(0.5-qx*qx-qy*qy);


		// PREDICT NEW VELOCITY
		State2[10]=vx+(awx)*dt;
		State2[11]=vy+(awy)*dt;
		State2[12]=vz+(awz-EARTH_GRAVITY)*dt;
		
		// partial derivatives of the new vx *********************************************************************************************		
		// d / d qx																												// d / d qy																				
		F.at<double>(10,0)=2*F.at<double>(7,0)/dt;												F.at<double>(10,1)=2*F.at<double>(7,1)/dt;
		// d / d qz																												// d / d qw
		F.at<double>(10,2)=2*F.at<double>(7,2)/dt;												F.at<double>(10,3)=2*F.at<double>(7,3)/dt;		
		F.at<double>(10,4)=0;																							F.at<double>(10,5)=0;												F.at<double>(10,6)=0;
		// d / d px																												// d / d py																	// d / d pz
		F.at<double>(10,7)=0;																							F.at<double>(10,8)=0;												F.at<double>(10,9)=0;
		// d / d vx																												// d / d vy																	// d / d vz
		F.at<double>(10,10)=1;																						F.at<double>(10,11)=0;											F.at<double>(10,12)=0;
		// d vx/ d abx																										// d / d aby																// d / d abz
		F.at<double>(10,13)=-dt*(1-2*qy*qy-2*qz*qz);											F.at<double>(10,14)=-dt*2*(qx*qy-qz*qw);		F.at<double>(10,15)=-dt*2*(qx*qz+qy*qw);

		// partial derivatives of the new vy *********************************************************************************************		
		// d vy/ d qx																											// d vy/ d qy																				
		F.at<double>(11,0)=2*F.at<double>(8,0)/dt;												F.at<double>(11,1)=2*F.at<double>(8,1)/dt;
		// d vy/ d qz																											// d vy/ d qw
		F.at<double>(11,2)=2*F.at<double>(8,2)/dt;												F.at<double>(11,3)=2*F.at<double>(8,3)/dt;
		
		F.at<double>(11,4)=0;																	F.at<double>(11,5)=0;												F.at<double>(11,6)=0;	// d / db
		F.at<double>(11,7)=0;																	F.at<double>(11,8)=0;												F.at<double>(11,9)=0;	// d / dp
		F.at<double>(11,10)=0;																F.at<double>(11,11)=1;											F.at<double>(11,12)=0;// d / dv
		// d vy/d abx																					// d vy/d aby																// d vy/ d abz
		F.at<double>(11,13)=-dt*2*(qx*qy+qz*qw);							F.at<double>(11,14)=-dt*(1-2*qx*qx-2*qz*qz);F.at<double>(11,15)=-dt*2*(qy*qz-qx*qw);

		// partial derivatives of the new vz *********************************************************************************************		
		// d vz/ d qx																											// d vz/ d qy																				
		F.at<double>(12,0)=2*F.at<double>(9,0)/dt;												F.at<double>(12,1)=2*F.at<double>(9,1)/dt;
		// d vz/ d qz																											// d vz/ d qw
		F.at<double>(12,2)=2*F.at<double>(9,2)/dt;												F.at<double>(12,3)=2*F.at<double>(9,3)/dt;
		
		F.at<double>(12,4)=0;																	F.at<double>(12,5)=0;												F.at<double>(12,6)=0;	// d / db
		F.at<double>(12,7)=0;																	F.at<double>(12,8)=0;												F.at<double>(12,9)=0;	// d / dp
		F.at<double>(12,10)=0;																F.at<double>(12,11)=0;											F.at<double>(12,12)=1;// d / dv
		// d vz/ d abx																				// d vz/ d aby															// d vz/ d abz
		F.at<double>(12,13)=-dt*2*(qx*qz-qy*qw);								F.at<double>(12,14)=-dt*2*(qy*qz+qx*qw);			F.at<double>(12,15)=-dt*(1-2*qx*qx-2*qy*qy);

		// PREDICTION OF THE NEW ACCEL BIAS - State2[13-15] REMAINS THE SAME ...
		
		// partial derivatives of the new abx *********************************************************************************************		
		F.at<double>(13,0)=0;		F.at<double>(13,1)=0;		F.at<double>(13,2)=0;	F.at<double>(13,3)=0;	
		F.at<double>(13,4)=0;		F.at<double>(13,5)=0;		F.at<double>(13,6)=0;
		F.at<double>(13,7)=0;		F.at<double>(13,8)=0;		F.at<double>(13,9)=0;
		F.at<double>(13,10)=0;	F.at<double>(13,11)=0;	F.at<double>(13,12)=0;
		F.at<double>(13,13)=1;	F.at<double>(13,14)=0;	F.at<double>(13,15)=0;
		// partial derivatives of the new aby *********************************************************************************************		
		F.at<double>(14,0)=0;		F.at<double>(14,1)=0;		F.at<double>(14,2)=0;	F.at<double>(14,3)=0;	
		F.at<double>(14,4)=0;		F.at<double>(14,5)=0;		F.at<double>(14,6)=0;
		F.at<double>(14,7)=0;		F.at<double>(14,8)=0;		F.at<double>(14,9)=0;
		F.at<double>(14,10)=0;	F.at<double>(14,11)=0;	F.at<double>(14,12)=0;
		F.at<double>(14,13)=0;	F.at<double>(14,14)=1;	F.at<double>(14,15)=0;
		// partial derivatives of the new abz *********************************************************************************************		
		F.at<double>(15,0)=0;		F.at<double>(15,1)=0;		F.at<double>(15,2)=0;	F.at<double>(15,3)=0;	
		F.at<double>(15,4)=0;		F.at<double>(15,5)=0;		F.at<double>(15,6)=0;
		F.at<double>(15,7)=0;		F.at<double>(15,8)=0;		F.at<double>(15,9)=0;
		F.at<double>(15,10)=0;	F.at<double>(15,11)=0;	F.at<double>(15,12)=0;
		F.at<double>(15,13)=0;	F.at<double>(15,14)=0;	F.at<double>(15,15)=1;
		
		// we have non-additive noise	- e.g. quaternion noise stems from gyroscope noise
		// take partial derivatives of the new state with respect to variables with process noise -  w,wb,a,ab	
		cv::Mat L(16,12,CV_64F);
				
		// partial derivative of the new qx	************************************************************************************************	
		L.at<double>(0,0)=-F.at<double>(0,4);	L.at<double>(0,1)=-F.at<double>(0,5);	L.at<double>(0,2)=-F.at<double>(0,6);	//df/dw
		L.at<double>(0,3)= F.at<double>(0,4);	L.at<double>(0,4)= F.at<double>(0,5);	L.at<double>(0,5)= F.at<double>(0,6);	//df/db
		L.at<double>(0,6)= 0;									L.at<double>(0,7)= 0;									L.at<double>(0,8)= 0;									//df/da
		L.at<double>(0,9)= 0;									L.at<double>(0,10)= 0;								L.at<double>(0,11)= 0;								//df/dab

		// partial derivative of the new qy	************************************************************************************************			
		L.at<double>(1,0)=-F.at<double>(1,4);	L.at<double>(1,1)=-F.at<double>(1,5);	L.at<double>(1,2)=-F.at<double>(1,6);	
		L.at<double>(1,3)= F.at<double>(1,4);	L.at<double>(1,4)= F.at<double>(1,5);	L.at<double>(1,5)= F.at<double>(1,6);
		L.at<double>(1,6)= 0;									L.at<double>(1,7)= 0;									L.at<double>(1,8)= 0;									//df/da
		L.at<double>(1,9)= 0;									L.at<double>(1,10)= 0;								L.at<double>(1,11)= 0;								//df/dab
		
		// partial derivative of the new qz	************************************************************************************************			
		L.at<double>(2,0)=-F.at<double>(2,4);	L.at<double>(2,1)=-F.at<double>(2,5);	L.at<double>(2,2)=-F.at<double>(2,6);
		L.at<double>(2,3)= F.at<double>(2,4);	L.at<double>(2,4)= F.at<double>(2,5);	L.at<double>(2,5)= F.at<double>(2,6);
		L.at<double>(2,6)= 0;									L.at<double>(2,7)= 0;									L.at<double>(2,8)= 0;									//df/da
		L.at<double>(2,9)= 0;									L.at<double>(2,10)= 0;								L.at<double>(2,11)= 0;								//df/dab
			
		// partial derivative of the new qw	************************************************************************************************			
		L.at<double>(3,0)=-F.at<double>(3,4);	L.at<double>(3,1)=-F.at<double>(3,5);	L.at<double>(3,2)=-F.at<double>(3,6);	
		L.at<double>(3,3)= F.at<double>(3,4);	L.at<double>(3,4)= F.at<double>(3,5);	L.at<double>(3,5)= F.at<double>(3,6);
		L.at<double>(3,6)= 0;									L.at<double>(3,7)= 0;									L.at<double>(3,8)= 0;									//df/da
		L.at<double>(3,9)= 0;									L.at<double>(3,10)= 0;								L.at<double>(3,11)= 0;								//df/dab

		// partial derivative of the new bx	************************************************************************************************			
		L.at<double>(4,0)= 0;				L.at<double>(4,1)= 0;				L.at<double>(4,2)= 0;			
		L.at<double>(4,3)= 1;				L.at<double>(4,4)= 0;				L.at<double>(4,5)= 0;
		L.at<double>(4,6)= 0;				L.at<double>(4,7)= 0;				L.at<double>(4,8)= 0;		//df/da
		L.at<double>(4,9)= 0;				L.at<double>(4,10)= 0;			L.at<double>(4,11)= 0;	//df/dab
		
		// partial derivative of the new by	************************************************************************************************			
		L.at<double>(5,0)= 0;				L.at<double>(5,1)= 0;				L.at<double>(5,2)= 0;			
		L.at<double>(5,3)= 0;				L.at<double>(5,4)= 1;				L.at<double>(5,5)= 0;
		L.at<double>(5,6)= 0;				L.at<double>(5,7)= 0;				L.at<double>(5,8)= 0;		//df/da
		L.at<double>(5,9)= 0;				L.at<double>(5,10)= 0;			L.at<double>(5,11)= 0;	//df/dab
		
		// partial derivative of the new bz	************************************************************************************************			
		L.at<double>(6,0)= 0;				L.at<double>(6,1)= 0;				L.at<double>(6,2)= 0;			
		L.at<double>(6,3)= 0;				L.at<double>(6,4)= 0;				L.at<double>(6,5)= 1;
		L.at<double>(6,6)= 0;				L.at<double>(6,7)= 0;				L.at<double>(6,8)= 0;		//df/da
		L.at<double>(6,9)= 0;				L.at<double>(6,10)= 0;			L.at<double>(6,11)= 0;	//df/dab
				
		// partial derivative of the new px	************************************************************************************************			
		L.at<double>(7,0)= 0;										L.at<double>(7,1)= 0;										L.at<double>(7,2)= 0;										//df/dw	
		L.at<double>(7,3)= 0;										L.at<double>(7,4)= 0;										L.at<double>(7,5)= 0;										//df/db
		L.at<double>(7,6)=-F.at<double>(7,13);	L.at<double>(7,7)=-F.at<double>(7,14);	L.at<double>(7,8)=-F.at<double>(7,15);	//df/da
		L.at<double>(7,9)= F.at<double>(7,13);	L.at<double>(7,10)=F.at<double>(7,14);	L.at<double>(7,11)= F.at<double>(7,15);	//df/dab

		// partial derivative of the new py	************************************************************************************************			
		L.at<double>(8,0)= 0;										L.at<double>(8,1)= 0;										L.at<double>(8,2)= 0;			
		L.at<double>(8,3)= 0;										L.at<double>(8,4)= 0;										L.at<double>(8,5)= 0;
		L.at<double>(8,6)=-F.at<double>(8,13);	L.at<double>(8,7)=-F.at<double>(8,14);	L.at<double>(8,8)=-F.at<double>(8,15);	//df/da
		L.at<double>(8,9)= F.at<double>(8,13);	L.at<double>(8,10)=F.at<double>(8,14);	L.at<double>(8,11)=F.at<double>(8,15);	//df/dab
	
		// partial derivative of the new pz	************************************************************************************************			
		L.at<double>(9,0)= 0;										L.at<double>(9,1)= 0;										L.at<double>(9,2)= 0;			
		L.at<double>(9,3)= 0;										L.at<double>(9,4)= 0;										L.at<double>(9,5)= 0;
		L.at<double>(9,6)=-F.at<double>(9,13);	L.at<double>(9,7)=-F.at<double>(9,14);	L.at<double>(9,8)=-F.at<double>(9,15);	//df/da
		L.at<double>(9,9)= F.at<double>(9,13);	L.at<double>(9,10)=F.at<double>(9,14);	L.at<double>(9,11)=F.at<double>(9,15);	//df/dab

		// partial derivative of the new vx	************************************************************************************************			
		L.at<double>(10,0)= 0;									L.at<double>(10,1)= 0;									L.at<double>(10,2)= 0;			
		L.at<double>(10,3)= 0;									L.at<double>(10,4)= 0;									L.at<double>(10,5)= 0;
		L.at<double>(10,6)=-F.at<double>(10,13);L.at<double>(10,7)=-F.at<double>(10,14);L.at<double>(10,8)=-F.at<double>(10,15);	//df/da
		L.at<double>(10,9)= F.at<double>(10,13);L.at<double>(10,10)=F.at<double>(10,14);L.at<double>(10,11)=F.at<double>(10,15);	//df/dab

		// partial derivative of the new vy	************************************************************************************************			
		L.at<double>(11,0)= 0;									L.at<double>(11,1)= 0;									L.at<double>(11,2)= 0;			
		L.at<double>(11,3)= 0;									L.at<double>(11,4)= 0;									L.at<double>(11,5)= 0;
		L.at<double>(11,6)=-F.at<double>(11,13);L.at<double>(11,7)=-F.at<double>(11,14);L.at<double>(11,8)=-F.at<double>(11,15);	//df/da
		L.at<double>(11,9)= F.at<double>(11,13);L.at<double>(11,10)=F.at<double>(11,14);L.at<double>(11,11)=F.at<double>(11,15);	//df/dab

		// partial derivative of the new vz	************************************************************************************************			
		L.at<double>(12,0)= 0;									L.at<double>(12,1)= 0;									L.at<double>(12,2)= 0;			
		L.at<double>(12,3)= 0;									L.at<double>(12,4)= 0;									L.at<double>(12,5)= 0;
		L.at<double>(12,6)=-F.at<double>(12,13);L.at<double>(12,7)=-F.at<double>(12,14);L.at<double>(12,8)=-F.at<double>(12,15);	//df/da
		L.at<double>(12,9)= F.at<double>(12,13);L.at<double>(12,10)=F.at<double>(12,14);L.at<double>(12,11)=F.at<double>(12,15);	//df/dab

		// partial derivative of the new abx	**********************************************************************************************			
		L.at<double>(13,0)= 0;									L.at<double>(13,1)= 0;									L.at<double>(13,2)= 0;			
		L.at<double>(13,3)= 0;									L.at<double>(13,4)= 0;									L.at<double>(13,5)= 0;
		L.at<double>(13,6)= 0;									L.at<double>(13,7)= 0;									L.at<double>(13,8)= 0;	//df/da
		L.at<double>(13,9)= 1;									L.at<double>(13,10)= 0;									L.at<double>(13,11)= 0;	//df/dab

		// partial derivative of the new aby	**********************************************************************************************			
		L.at<double>(14,0)= 0;									L.at<double>(14,1)= 0;									L.at<double>(14,2)= 0;			
		L.at<double>(14,3)= 0;									L.at<double>(14,4)= 0;									L.at<double>(14,5)= 0;
		L.at<double>(14,6)= 0;									L.at<double>(14,7)= 0;									L.at<double>(14,8)= 0;	//df/da
		L.at<double>(14,9)= 0;									L.at<double>(14,10)= 1;									L.at<double>(14,11)= 0;	//df/dab

		// partial derivative of the new abz	**********************************************************************************************			
		L.at<double>(15,0)= 0;									L.at<double>(15,1)= 0;									L.at<double>(15,2)= 0;			
		L.at<double>(15,3)= 0;									L.at<double>(15,4)= 0;									L.at<double>(15,5)= 0;
		L.at<double>(15,6)= 0;									L.at<double>(15,7)= 0;									L.at<double>(15,8)= 0;	//df/da
		L.at<double>(15,9)= 0;									L.at<double>(15,10)= 0;									L.at<double>(15,11)= 1;	//df/dab
			
		// predict new covariance
		KalmanP2=F*KalmanP2*F.t()+L*KalmanQ2*L.t();
		
	}
	
	void KalmanUpdateGPS(double x, double y, double z)
	{		
		if (update2)
		{											
			cv::Mat S=H_GPS*KalmanP2*H_GPS.t()+KalmanR2;
			cv::Mat Sinv=S.inv(cv::DECOMP_CHOLESKY);
			cv::Mat K=KalmanP2*H_GPS.t()*Sinv;

			cv::Mat Y(3,1,CV_64F);// POSITION RESIDUAL...
		
			// GLOBAL GPS UPDATE
			Y.at<double>(0,0)=x-State2[7];
			Y.at<double>(1,0)=y-State2[8];
			Y.at<double>(2,0)=z-State2[9];
		
			cv::Mat KY=K*Y;
		
			// UPDATE THE STATE...
			for (int i=0;i<16;i++)
			{
				State2[i]+=KY.at<double>(i,0);
			}

			// UPDATE THE COVARIANCE MATRIX	
		  cv::Mat KHP=K*H_GPS*KalmanP2;
			KalmanP2-=KHP;
		}				
								
		qtSolution2.at<double>(0)=State2[0];
		qtSolution2.at<double>(1)=State2[1];
		qtSolution2.at<double>(2)=State2[2];
		qtSolution2.at<double>(3)=State2[3];				
	}
};

#endif
