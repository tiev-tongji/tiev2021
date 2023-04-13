#include <bits/stdc++.h>
#include "angle.h"

using namespace std;

int main(){
    TiEV::Angle test;
    test.setByDegree(180, 90, TiEV::Angle::CLOCKWISE);
    cout << test.getDegree() << ' ' << test.getRad() << endl;
    return 0;
}
