#include "stdafx.h"
#include <direct.h>
#include <iostream>
#include <fstream>
#include <thread>
#include <time.h>
#include <mutex>
#include <chrono>
#include "deviceclass.h"
#include "kalman3.h";
#include "SBPClasses.h";
#include "Serial.h"
#include <xsens/xsportinfoarray.h>
#include <xsens/xsdatapacket.h>
#include <xsens/xstime.h>
#include <xsens/legacydatapacket.h>
#include <xsens/int_xsdatapacket.h>
#include <xsens/enumerateusbdevices.h>
using namespace std;


Kalman2 ekf;
bool workerThreadRun, GPSThreadRun, changeGPS;
NEDPosition position;
XsEuler eulerAngles;
clock_t timer = clock();

mutex position_mutex;
char foldername[50];
ofstream fout;
ofstream fout2;

NEDPosition parseNED(byte buffer[])
{
	unsigned int tow = 0;
	memcpy(&tow, buffer + 6, 4);
	int north = 0;
	memcpy(&north, buffer + 10, 4);
	int east = 0;
	memcpy(&east, buffer + 14, 4);
	int down = 0;
	memcpy(&down, buffer + 18, 4);
	//cout  << tow << " N:" << north << " E:" << east << " D:" << down << endl;
	
	//if (tow - position.TimeOfWeek != 100) cout << position.TimeOfWeek << " " << tow << endl;
	
	NEDPosition ned;
	ned.TimeOfWeek = tow%86400000;
	ned.North = north;
	ned.East = east;
	ned.Down = down;
	return ned;
}

GPSTime parseTime(byte buffer[])
{
	unsigned int tow = 0;
	memcpy(&tow, buffer + 8, 4);
	int nstime = 0;
	memcpy(&nstime, buffer + 12, 4);
	GPSTime timeOfWeek;
	timeOfWeek.tow = tow;
	timeOfWeek.nsTime = nstime;
	position.TimeOfWeek = tow % 86400000;
	return timeOfWeek;
}



void GPSThread();
void workerThread();
int main()
{
	workerThreadRun = true;
	GPSThreadRun = true;
	changeGPS = false;
	
	position.North = 0;
	position.East = 0;
	position.Down = 0;
	thread first(workerThread);
	thread second(GPSThread);
	char inputChar;
	cin >> inputChar;
	while (inputChar != 'e') {
		if (inputChar == 'p') {
			double x, y, z;
			ekf.getPosition(x, y, z);
			cout << x << " " << y << " " << z << endl;
		}
		else if (inputChar == 'o') {
			cout << eulerAngles.y() << " " << -eulerAngles.z() << " " << -eulerAngles.x() << endl;
		}
		cin >> inputChar;
	}
	
	workerThreadRun = false;
	GPSThreadRun = false;
	first.join();
	second.join();
	//cout << "Done" << endl;

	return 0;
}


void workerThread()
{
	DeviceClass device;
	//ofstream fout2("positions2.txt");

	try
	{
		// Scan for connected USB device
		//cout << "Searching for USB devices..." << endl;

		XsPortInfoArray portInfoArray;
		xsEnumerateUsbDevices(portInfoArray);
		if (!portInfoArray.size())
		{
			std::string portName;
			int baudRate;
#ifdef WIN32
			//std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. COM1): " <<
#else
			//std::cout << "No USB Motion Tracker found." << std::endl << std::endl << "Please enter COM port name (eg. /dev/ttyUSB0): " <<
#endif
				//std::endl;
				//std::cin >> portName;
				//std::cout << "Please enter baud rate (eg. 115200): ";
				//std::cin >> baudRate;

				// Scan for connected USB devices
			//cout << "No USB devices - trying COM6 - 921600..." << endl;

			//TODO - implement selection of different ports and rates for serial xsens
			portName = "COM6";
			baudRate = 921600;

			XsPortInfo portInfo(portName, XsBaud::numericToRate(baudRate));
			portInfoArray.push_back(portInfo);
		}

		// Use the first detected device
		XsPortInfo mtPort = portInfoArray.at(0);

		// Open the port with the detected device
		//cout << "Opening port..." << endl;
		if (!device.openPort(mtPort))
		{
			//cout << "Can't find Port !" << endl;
			return;
		}


		if (!device.gotoConfig()) // Put the device into configuration mode before configuring the device
		{
			//cout << "Could not put device into configuration mode. Aborting." << endl;

			return;
		}

		// Request the device Id to check the device type
		mtPort.setDeviceId(device.getDeviceId());

		// Check if we have an MTi / MTx / MTmk4 device
		if (!mtPort.deviceId().isMtix() && !mtPort.deviceId().isMtMk4())
		{
			//cout << "No MTi / MTx / MTmk4 device found. Aborting." << endl;

			return;
		}
		else
		{
			//std::cout << "Found a device with id: " << mtPort.deviceId().toString().toStdString() << " @ port: " << mtPort.portName().toStdString() << ", baudrate: " << mtPort.baudrate() << std::endl;
;

			// na 400 Hz vremenski intervali primanja podataka više nisu jednoliko rasporeðeni !!!
			// teèni prikaz na oculus rift se gubi preko 200 Hz za USB odnosno preko 50 Hz za serijski kabel !!!???
			// za sinkronizaciju sa locatom nam treba serijski kabel !!!

			// TODO - install serial cable and implement hardware sync with locata !
			// TODO - solve near real-time data acquisition on serial cable on at least 200 Hz - IMPOSSIBLE ???

			// usb - 200 Hz O.K.
			// usb - 400 Hz - packets of two !!

			// serial - 50 Hz - O.K.
			// serial - 100 Hz - pacets of two !!!
			// serial - 200 Hz - pacets of four !!!
			uint16_t frequency = 200;// select between 50,100,200,400

			if (mtPort.deviceId().isMtix())
			{
			}
			else if (mtPort.deviceId().isMtMk4())
			{
				ekf.setRate(frequency);
				XsOutputConfiguration accel(XDI_Acceleration, frequency);
				XsOutputConfiguration gyro(XDI_RateOfTurn, frequency);
				XsOutputConfiguration status(XDI_StatusWord, frequency);
				XsOutputConfiguration orientation(XDI_EulerAngles, frequency);
				XsOutputConfigurationArray configArray;
				configArray.push_back(accel);
				configArray.push_back(gyro);
				configArray.push_back(status);
				configArray.push_back(orientation);
				if (!device.setOutputConfiguration(configArray))
				{
				//cout << "Could not configure MTmk4 device. Aborting." << endl;
					return;
				}
			}
			else
			{
				return ;
			}

			XsByteArray data;
			XsMessageArray msgs;

			bool bFirstRun = true;
			int display_rate = frequency / 10;	// set display rate to 10 Hz
			device.gotoMeasurement();
			while (workerThreadRun)
			{
				device.readDataToBuffer(data);
				device.processBufferedData(data, msgs);
				for (XsMessageArray::iterator it = msgs.begin(); it != msgs.end(); ++it)
				{
					// Retrieve a packet
					XsDataPacket packet;
					if ((*it).getMessageId() == XMID_MtData)
					{
					}
					else if ((*it).getMessageId() == XMID_MtData2)
					{
						packet.setMessage((*it));
						packet.setDeviceId(mtPort.deviceId());

						// Stop the watch and restart it again immediately
						// measure data rate - does not work above 200 Hz due to uneven periods of reception !!!
						//float ElapsedTime = (float)This->m_StopWatch.Stop(true);

							//debug - display distribution of dt - should be approximately equal times !
							//pTemp=new CString;
							//pTemp->Format("%.1f ms",ElapsedTime*1000);
							//This->PostMessage(WM_ADD_STRING,(WPARAM)pTemp);
						if (bFirstRun)
						{
							int64_t ts = XsTimeStamp::now().msTime();
							sprintf(foldername, "Log%I64d", ts);
							mkdir(foldername);
							string firstname = foldername + (string)("/GPSPos.txt");
							string secname = foldername + (string)("/Xsens.txt");
							fout.open(firstname);
							fout2.open(secname);
						}
						XsVector CalAccel = packet.calibratedAcceleration();
						//CalAcc[0],CalAcc[1],CalAcc[2]
						eulerAngles = packet.orientationEuler();
						XsVector CalGyro = packet.calibratedGyroscopeData();
						//CalGyro[0],CalGyro[1],CalGyro[2]
						uint32_t status = packet.status();
						fout2 << CalAccel[0] << "," << CalAccel[1] << "," << CalAccel[2] << "," << CalGyro[0] << "," << CalGyro[1] << "," << CalGyro[2] << "," << eulerAngles.x() << "," << eulerAngles.y() << "," << eulerAngles.z() << "," << endl;
						int64_t ts = XsTimeStamp::now().msTime();
						fout2 << ts << endl;
						//gettimeof(&tp, NULL);
						//long long mslong = (long long)tp.tv_sec * 1000L + tp.tv_usec / 1000; //get current timestamp in milliseconds
						//std::cout << mslong << std::endl;
						if (bFirstRun)
						{
							while (!changeGPS) Sleep(1);
							position_mutex.lock();
							ekf.InitState(CalAccel[0], CalAccel[1], CalAccel[2], CalGyro[0], CalGyro[1], CalGyro[2], position.North / 1000.0, -position.East / 1000.0, -position.Down / 1000.0);
							bFirstRun = false;
							changeGPS = false;
							position_mutex.unlock();
						}
						ekf.KalmanPredict2(CalGyro[0], CalGyro[1], CalGyro[2], CalAccel[0], CalAccel[1], CalAccel[2]);
						if (status & 0x00200000 == 0x00200000 && changeGPS)
						{
							//new measurement from locata !!!

							//TODO-implement some thread-safe transfer of the last locata measurement here
							//TODO-compensate for positive/negative delay
							//TODO-transform Locata position into IMU position before the update !
							int64_t ts = XsTimeStamp::now().msTime();
							fout << position.North << "," << position.East << "," << position.Down << endl;
							fout << ts << endl;
							position_mutex.lock();
							ekf.KalmanUpdateGPS(position.North / 1000.0, -position.East / 1000.0, -position.Down / 1000.0);
							changeGPS = false;
							position_mutex.unlock();

							//debug - check if hardware conection is alive - should also be enabled in xsens configuration !
							//pTemp=new CString;
							//pTemp->Format("SyncIn: status: %.4X",status);
							//This->PostMessage(WM_ADD_STRING,(WPARAM)pTemp);
						}
						//double x, y, z;
						//ekf.getPosition(x, y, z);
						//fout2 << position.East/1000.0 << " " << position.North/1000.0 << endl;
					}
				}
				msgs.clear();
				XsTime::msleep(0);
			}
		}//we have an MTi / MTx / MTmk4 device

		// Close port
		//cout << "Closing port..." << endl;
		device.close();
		fout2.close();
		fout.close();
	}
	catch (...)
	{
	}
}

void GPSThread()
{
	try
	{
		//cout << "Opening com port" << endl;
		string commPortName("COM5");
		Serial serial(commPortName, 1000000);
		//cout << "Port opened" << endl;


		byte buffer[10000];

		//cout << "Reading from the serial port: " << endl;
		int charsRead = 100;
		vector<byte> bytesRead;
		while (GPSThreadRun)
		{
			charsRead = serial.read(buffer, 10000);
			while (charsRead > 0)
			{
				bytesRead.insert(bytesRead.end(), buffer, buffer + charsRead);
				//Sleep(20);
				charsRead = serial.read(buffer, 10000);
			}
			int bytesSize = (int)bytesRead.size();
			int numberOfBytes = 0;
			for (int i = 0; i < bytesSize - 5; i++)
			{
				if (bytesRead[i] == 0x55)
				{
					int messageType = (int)bytesRead.at(i + 1) + (int)bytesRead.at(i + 2) * 256;
					int msgLength = (int)bytesRead.at(i + 5);
					if (i + 8 + msgLength >= bytesSize) break;
					switch (messageType)
					{
					case 515:
					{
						position_mutex.lock();
						position = parseNED(&bytesRead[i]);
						changeGPS = true;
						position_mutex.unlock();
						break;
					}
					case 256:
					{
						parseTime(&bytesRead[i]);
						break;
					}
					default:
						break;
					}
					numberOfBytes += 8 + msgLength;
					i = i + msgLength + 7;
				}
				else 
				{
					numberOfBytes++;
				}
			}
			//bytesRead.clear();
			bytesRead.erase(bytesRead.begin(), bytesRead.begin() + numberOfBytes);
		}

	}
	catch (const char *msg)
	{
		//cout << msg << endl;
	}



	return;
}