#ifndef FITDATA_H
#define FITDATA_H
#include "mystuff.h"
#include <vector>
//
//						 data.h
//

/************************************************************************
			DECLARATION OF THE CLASS DATA
************************************************************************/
// Class for Data
// A data has 5 fields: 
//       n (of type int), the number of data points 
//       X and Y (arrays of type reals), arrays of x- and y-coordinates
//       meanX and meanY (of type reals), coordinates of the centroid (x and y sample means)
class FitData
{
public:

	int n;
	vector<pair<double, double> > points; 		//space is allocated in the constructors
	reals meanX, meanY;

	// constructors
	FitData();
	FitData(int N);
	FitData(vector<pair<double, double> > &pointsData);

	// routines
	void means(void);
	void center(void);
	void scale(void);
	void print(void);

	// destructors
	~FitData();
};


/************************************************************************
			BODY OF THE MEMBER ROUTINES
************************************************************************/
// Default constructor
FitData::FitData()
{
}

// Constructor with assignment of the field N
FitData::FitData(int N)
{
	n=N;

	for (int i=0; i<n; i++)
	{
		points.push_back(make_pair(0, 0));
	}
}

// Constructor with assignment of each field
FitData::FitData(vector<pair<double, double> > &pointsData)
{
	n=pointsData.size();

	points = pointsData;
}

// Routine that computes the x- and y- sample means (the coordinates of the centeroid)

void FitData::means(void)
{
	meanX=0.; meanY=0.;
	
	for (int i=0; i<n; i++)
	{
		meanX += points[i].first;
		meanY += points[i].second;
	}
	meanX /= n;
	meanY /= n;
}

// Routine that centers the data set (shifts the coordinates to the centeroid)

void FitData::center(void)
{
	reals sX=0.,sY=0.;  
	int i;
	
	for (i=0; i<n; i++)
	{
		sX += points[i].first;
		sY += points[i].second;
	}
	sX /= n;
	sY /= n;
	
	for (i=0; i<n; i++)
	{
		points[i].first -= sX;
		points[i].second -= sY;
	}
	meanX = 0.;
	meanY = 0.;
}

// Routine that scales the coordinates (makes them of order one)

void FitData::scale(void)
{
	reals sXX=0.,sYY=0.,scaling;  
	int i;
	
	for (i=0; i<n; i++)
	{
		sXX += points[i].first*points[i].first;
		sYY += points[i].second*points[i].second;
	}
	scaling = sqrt((sXX+sYY)/n/Two);
	
	for (i=0; i<n; i++)
	{
		points[i].first /= scaling;
		points[i].second /= scaling;
	}
}

// Printing routine

void FitData::print(void)
{
	cout << endl << "The data set has " << n << " points with coordinates :"<< endl;
	
	for (int i=0; i<n-1; i++) cout << setprecision(7) << "(" << points[i].first << ","<< points[i].second << "), ";
	
	cout << "(" << points[n-1].first << ","<< points[n-1].second << ")\n";
}

// Destructor
FitData::~FitData()
{
}

#endif
