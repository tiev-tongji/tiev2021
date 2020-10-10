#ifndef FITCIRCLE_H
#define FITCIRCLE_H

//
//						 circle.h
//
/************************************************************************
			DECLARATION OF THE CLASS CIRCLE
************************************************************************/
// Class for Circle
// A circle has 7 fields: 
//     a, b, r (of type reals), the circle parameters
//     s (of type reals), the estimate of sigma (standard deviation)
//     g (of type reals), the norm of the gradient of the objective function
//     i and j (of type int), the iteration counters (outer and inner, respectively)

class FitCircle
{
public:

	// The fields of a Circle
	reals a, b, r, s, g, Gx, Gy;
	int i, j;

	// constructors
	FitCircle();
	FitCircle(reals aa, reals bb, reals rr);

	// routines
	void print(void);

	// no destructor we didn't allocate memory by hand.
};


/************************************************************************
			BODY OF THE MEMBER ROUTINES
************************************************************************/
// Default constructor

FitCircle::FitCircle()
{
	a=0.; b=0.; r=1.; s=0.; i=0; j=0;
}

// Constructor with assignment of the circle parameters only

FitCircle::FitCircle(reals aa, reals bb, reals rr)
{
	a=aa; b=bb; r=rr;
}

// Printing routine

void FitCircle::print(void)
{
	cout << endl;
	cout << setprecision(10) << "center (" <<a <<","<< b <<")  radius "
		 << r << "  sigma " << s << "  gradient " << g << "  iter "<< i << "  inner " << j << endl;
}

#endif
