#include "CircleFitByTaubin.h"
#include <vector>

int main(){
	double tX[5] = {0, 1, 2, 3, 4};
	double tY[5] = {0, 0, 0, 0, 0};
	vector<pair<double, double> > fitDatas;
	for(int i = 0; i < 5; i++) {
		fitDatas.push_back(make_pair(tX[i], tY[i]));
	}
	FitData fitData(fitDatas);
	FitCircle circle = CircleFitByTaubin(fitData);
	fitData.print();
	circle.print();
	return 0;
}
