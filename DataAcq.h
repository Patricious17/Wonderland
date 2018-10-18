#ifndef DATAACK_H
#define DATAACK_H

//#include "kalman3.h"
//#include <string>
//#include <opencv2/highgui/highgui.hpp>

using namespace std;

struct sample {
	bool active;
	double value;
};

struct GPSset {
	double tG;
	sample N;
	sample E;
	sample D;
};

struct XSset {
	double tX;
	sample accX;
	sample accY;
	sample accZ;
	sample gyroX;
	sample gyroY;
	sample gyroZ;
	sample EAX;
	sample EAY;
	sample EAZ;
};


class DAcq
{

	public:
		XSset XSdata;
		GPSset GPSdata;
		// modes of data acquisition: 
		// 1) from file with predefined structure of data that is compliant with GNU plot
		// 2) from sensors:: TODO

		string md; 

	DAcq(string mode)
	{
		if (mode == "from file" || mode == "from sensors"){			
			GPSdata = { 0, 0, false, 0, false, 0, false };			
			XSdata = { 0, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false, 0, false };
		}
		else {
			cout << "selected mode does not exist, object remains uninitialized (possible are: 'from file' and 'from sensors')" << endl;
			return;
		}		
	}

	~DAcq(){}

	void ActivateData(char GPSword[3], char XSword[9]) {

		bool *pG = &GPSdata.N.active;
		bool *pX = &XSdata.accX.active;

		for (int k = 0; k <= 2; k++) {

			switch (GPSword[k])
			{
			case '1':
				*(pG + k*(sizeof(sample))) = true;
				break;
			case '0':
				*(pG + k*(sizeof(sample))) = false;
				break;
			default:
				cout << "Unexpected characters in activation word of data acq. for GPS ('0' and '1' are expected)" << endl;
				return;
			}
		}

		for (int k = 0; k <= 8; k++) {

			switch (XSword[k])
			{
			case '1':
				*(pX + k*(sizeof(sample))) = true;
				break;
			case '0':
				*(pX + k*(sizeof(sample))) = false;
				break;
			default:
				cout << "Unexpected characters in activation word of data acq. for XS ('0' and '1' are expected)" << endl;
				return;
			}
		}
	}

	bool GetLine(ifstream *source, char src) {
		double garbage=0;
		if(src == 'g')		
		{
			if (!(*source >> GPSdata.tG)) if ((*source).eof) return false; // checks for EOF, double if to make sure that EOF flag sets

			if (GPSdata.N.active) *source >> GPSdata.N.value;
			if (GPSdata.E.active) *source >> GPSdata.E.value;
			if (GPSdata.D.active) *source >> GPSdata.D.value;
		}
		else if (src == 'x') 
		{			
			if (!(*source >> XSdata.tX)) if ((*source).eof)  return false;  // checks for EOF, double if to make sure that EOF flag sets

			if (XSdata.accX.active)  *source >> XSdata.accX.value; 
			if (XSdata.accY.active)  *source >> XSdata.accY.value; 
			if (XSdata.accZ.active)  *source >> XSdata.accZ.value; 
			if (XSdata.gyroX.active) *source >> XSdata.gyroX.value;
			if (XSdata.gyroY.active) *source >> XSdata.gyroY.value;
			if (XSdata.gyroZ.active) *source >> XSdata.gyroZ.value;
			if (XSdata.EAX.active)   *source >> XSdata.EAX.value;
			if (XSdata.EAY.active)   *source >> XSdata.EAY.value;
			if (XSdata.EAZ.active)   *source >> XSdata.EAZ.value;		
			
		}
		else { cout << "wrong type of source in GetLine" << endl; return false;}

		return true;
	}	
};

#endif