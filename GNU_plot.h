#ifndef GNU_PLOT_H_
#define GNU_PLOT_H_

//#include "kalman3.h"
#include <string>
//#include <opencv2/highgui/highgui.hpp>
using namespace std;

class GNU_plot
{
	struct pSpec {
		FILE *pipe;
		string title;
		string xlabel;
		string ylabel;
	};

	struct gSpec {
		FILE *gpipe;
		string fName;
		string gName;
		string dMode;
		int cNum;
		int resolP;				
	};

	char buffer[100] = { 0 };
	char command[500] = { 0 };		
	
public:	

	pSpec plotSpec;
	gSpec *p2graphs = new gSpec[5];
	int numLines;

	GNU_plot(FILE *Pipe,string Title, string Xlabel, string Ylabel)
	{		
		plotSpec.title = Title;
		plotSpec.xlabel = Xlabel;
		plotSpec.ylabel = Ylabel;
		plotSpec.pipe = Pipe;
		numLines = 0;				
	}
	~GNU_plot() {}

	void AddLine(string fileName, string lineName, string drawMode, int colNum, int resolution)
	{
		if (numLines >= 5)
		{
			cout << "maximum number of lines reached\n" << endl;
			return;
		}
		else
		{
			(p2graphs + numLines)->cNum   = colNum;
			(p2graphs + numLines)->fName  = fileName;
			(p2graphs + numLines)->resolP = resolution;
			(p2graphs + numLines)->dMode = drawMode;
			numLines++;
		}
	}
		
	void Plot(int startP, int endP, string plotReplot)
	{		
		command[0] = '\0';
		buffer[0] = '\0';

		for (int j = 0; j < numLines;j++)
		{	

			if (j == 0 && (plotReplot=="plot"))  
				strcat_s(command, sizeof(command), "plot ");				 
			else if (plotReplot == "replot")
				strcat_s(command, sizeof(command), "replot ");				

			sprintf_s(buffer, sizeof(buffer), "'%s' using 1:%d every %d::%d::%d %s\n", ((p2graphs +j)->fName).c_str(),(p2graphs +j)->cNum, (p2graphs +j)->resolP, startP, endP, ((p2graphs+j)->dMode).c_str());
			strcat_s(command, sizeof(command), buffer);
		}		
		cout << command << endl;		
		fprintf(plotSpec.pipe, "%s\n", command);		
		
		fflush(plotSpec.pipe);
	}

	void RePlot(int startP, int endP)
	{
		command[0] = '\0';
		buffer[0] = '\0';

		for (int j = 0; j < numLines; j++)
		{
			strcat_s(command, sizeof(command), "replot ");			

			sprintf_s(buffer, sizeof(buffer), "'%s' using 1:%d every %d::%d::%d %s\n", ((p2graphs + j)->fName).c_str(), (p2graphs + j)->cNum, (p2graphs + j)->resolP, startP, endP, ((p2graphs + j)->dMode).c_str());
			strcat_s(command, sizeof(command), buffer);
		}
		cout << command << endl;
		fprintf(plotSpec.pipe, "%s\n", command);

		fflush(plotSpec.pipe);
	}
};

#endif