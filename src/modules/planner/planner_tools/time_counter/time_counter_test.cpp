#include <bits/stdc++.h>
#include "time_lock/time_lock.h"
#include "time_counter.h"

using namespace std;
using namespace TiEV;

int main(){
	TimeLock test_lock;
	test_lock.lock(5000);
	TimeCounter test_counter;
	test_counter.start();
	int i = 0;
	while (test_lock.isLock()) {
		cout << test_counter.duration() << endl;
	}
	return 0;
}
