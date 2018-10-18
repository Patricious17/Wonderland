#include <iostream>
#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <Windows.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "GNU_plot.h"
#include "kalman3.h"
#include "DataAcq.h"

using namespace std;

#ifdef WIN32
#define GNUPLOT_NAME "gnuplot -persist"
#else 
#define GNUPLOT_NAME "gnuplot"
#endif

int main(int argc, char** argv) {	
	
	{		
		/*
		wchar_t exe_Path[MAX_PATH];
		GetModuleFileName(NULL, exe_Path, MAX_PATH);
		cout << exe_Path;
		cin.get();
		cin.get();
		*/

	/*INITIALIZATION*/
	/****************************************************************************************************************/

	/*DATA ACQ. INIT.*/
	/*errno_t err;

	FILE *wPipeG;
	err = fopen_s(&wPipeG, "f2writeG.txt", "w+");
	if (err == 0)
	{
	printf("The file 'f2writeG.txt' was opened\n");
	}
	else
	{
	printf("The file 'f2writeG.txt' was not opened\n");
	return -1;
	}

	FILE *wPipeX;
	err = fopen_s(&wPipeX, "f2writeX.txt", "w+");
	if (err == 0)
	{
	printf("The file 'f2writeX.txt' was opened\n");
	}
	else
	{
	printf("The file 'f2writeX.txt' was not opened\n");
	return -1;
	}

	FILE *rPipeG;
	err = fopen_s(&rPipeG, "GPS.txt", "r");
	if (err == 0)
	{
	printf("The file 'GPS.txt' was opened\n");
	}
	else
	{
	printf("The file 'GPS.txt' was not opened\n");
	return -1;
	}

	FILE *rPipeX;
	err = fopen_s(&rPipeX, "Xs.txt", "r");
	if (err == 0)
	{
	printf("The file 'Xs.txt' was opened\n");
	}
	else
	{
	printf("The file 'Xs.txt' was not opened\n");
	return -1;
	}*/
		ifstream GPSsource("GPS.txt");
		ifstream XSsource("Xs.txt");
		ofstream EKFdest("EKFdest.txt");
		ofstream GPSdest("GPSdest.txt");

		string sourceMode = "from file";
		DAcq dS(sourceMode); // dS --> data source
		dS.ActivateData("111", "111111111");

	/*PLOT INIT.*/
		FILE* pipe = _popen("C:/gnuplot/bin/gnuplot.exe", "w");
		if (pipe == NULL)
		{
			cout << "Could not open pipe to GNUplot\n" << endl; return -1;
		}
		fprintf(pipe, "set term wxt\n");		
		
		GNU_plot gpsPl(pipe, "GPSdest.txt", "time/ms", "value");	
		gpsPl.AddLine("GPSdest.txt", "z_gps", "with points", 2, 1);
		GNU_plot ekfPl(pipe, "EKFdest.txt", "time/ms", "value");
		ekfPl.AddLine("EKFdest.txt", "z_est", "with lines", 2, 1);		
		
	/*EKF INIT.*/
		Kalman2 ekf;
		uint16_t frequency = 200;
		ekf.setRate(frequency);

	/*TRB. INIT.*/
		int trbValueRange = 1000;
		int gbQ = trbValueRange / 10; int abQ = trbValueRange / 10;
		int aQ = trbValueRange / 10;  int gQ = trbValueRange / 10;
		int pR = trbValueRange / 10;

		cv::namedWindow("Trackbars", cv::WINDOW_NORMAL);
		cv::createTrackbar("gyroBQ", "Trackbars", &gbQ, trbValueRange);
		cv::createTrackbar("accBQ",  "Trackbars", &abQ, trbValueRange);
		cv::createTrackbar("gyroQ",  "Trackbars", &gQ,  trbValueRange);
		cv::createTrackbar("accQ",   "Trackbars", &aQ,  trbValueRange);
		cv::createTrackbar("pR",     "Trackbars", &pR,  trbValueRange);

	/*OTHER UTILITIES*/
		double t = 0;
		double tdiff = 6; //Xs and GPS time of occurance difference (t_gps - t_xs)
		bool bFirstRun = true;
		bool readG = false;
		bool readX = false;

		int horizon = 250;
		int numEKF = 0;
		int numGPS = 0;
		int startP = 0;
																								
	/*INITIALIZATION OVER********************************************************************************************/
								
		bool plotIt = true;
		while (1)
		{

			if (sourceMode == "from file")
			{
				/*DATA ACQUISITION*/

				if (bFirstRun) { dS.GetLine(&GPSsource, 'g'); dS.GetLine(&XSsource, 'x'); }

				if (readG)
				{
					if (dS.GetLine(&GPSsource, 'g')) { readG = false; }
					else { cout << "Seemingly, reading file failed... possibly, EOF flag was set" << endl;  break; }
				}
				if (readX)
				{
					if (dS.GetLine(&XSsource,  'x')) { readX = false; }
					else { cout << "Seemingly, reading file failed... possibly, EOF flag was set" << endl;  break; }
				}

				if (dS.XSdata.tX < dS.GPSdata.tG + tdiff)
				{
					t = dS.XSdata.tX;
					EKFdest << t;
				}

				else
				{
					t = dS.GPSdata.tG + tdiff;
					EKFdest << t;
					GPSdest << t;
					GPSdest << "    " << 0.001*dS.GPSdata.N.value << endl;
					//GPSdest << "    " << 0.001*dS.GPSdata.E.value << endl;
					//GPSdest << "    " << 0.001*dS.GPSdata.D.value << endl;
					numGPS++;
				}

				/*EKF INITIALIZATION*/
				if (bFirstRun)
				{
					ekf.InitState(dS.XSdata.accX.value, dS.XSdata.accY.value, dS.XSdata.accZ.value,
						dS.XSdata.gyroX.value, dS.XSdata.gyroY.value, dS.XSdata.gyroZ.value,
						0.001*dS.GPSdata.N.value, -0.001*dS.GPSdata.E.value, -0.001*dS.GPSdata.D.value);
					bFirstRun = false;
				}

				/*KALMAN BASED PREDICTION*/
				if (!readX && dS.XSdata.tX == t)
				{
					ekf.KalmanPredict2(dS.XSdata.accX.value, dS.XSdata.accY.value, dS.XSdata.accZ.value,
						dS.XSdata.gyroX.value, dS.XSdata.gyroY.value, dS.XSdata.gyroZ.value);
					readX = true;
				}

				if (!readG && (dS.GPSdata.tG + tdiff == t))
				{
					ekf.KalmanUpdateGPS(0.001*dS.GPSdata.N.value, -0.001*dS.GPSdata.E.value, -0.001*dS.GPSdata.D.value);
					readG = true;
				}

				numEKF++;
				EKFdest << "    " << ekf.State2[7];
				EKFdest << "    " << -ekf.State2[8];
				EKFdest << "    " << -ekf.State2[9];
				EKFdest << endl;
			}

			else if (sourceMode == "from sensors")
			{
				//TODO
			}

			else { cout << "non-existant data source mode" << endl; return -1; }

			if (numEKF == startP + horizon)
			{
				if (plotIt)
				{
					ekfPl.Plot(startP, startP + horizon, "plot");
					gpsPl.Plot(1, numGPS, "replot");
					//cv::waitKey(4000);
				}

				startP = startP + horizon + 1;

				//KALMAN FILTER RETUNING			

				ekf.setAccelPredictQ(aQ*0.001, aQ*0.001, aQ*0.001);
				ekf.setAccelBPredictQ(abQ*0.00001, abQ*0.00001, abQ*0.00001);
				ekf.setGyroPredictQ(gQ*0.00001, gQ*0.00001, gQ*0.00001);
				ekf.setGyroBPredictQ(gbQ*0.000000001, gbQ*0.000000001, gbQ*0.000000001);
				ekf.setPositionUpdateR(pR*0.00000001, pR*0.00000001, pR*0.00000001);
				cout << ekf.KalmanR2.at<double>(0, 0) << endl
					<< ekf.KalmanQ2.at<double>(9, 9) << endl
					<< ekf.KalmanQ2.at<double>(6, 6) << endl
					<< ekf.KalmanQ2.at<double>(3, 3) << endl
					<< ekf.KalmanQ2.at<double>(0, 0) << endl;
			}
			
		}		

		ekfPl.Plot(startP, startP + horizon, "plot");
		gpsPl.Plot(1, numGPS, "replot");
		system("PAUSE");

		GPSsource.close();
		XSsource.close();
		GPSdest.close();
		EKFdest.close();

		_pclose(pipe);
		return 0;
	}		
}
