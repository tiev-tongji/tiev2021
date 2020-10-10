#include <bits/stdc++.h>
#include "time_lock.h"

using namespace std;
using namespace TiEV;

int main(){
	TimeLock test_lock;
	test_lock.lock(5000);
	int i = 0;
	while (test_lock.isLock()) {
		cout << (i++) << endl;
	}
	return 0;
}
