#include "visualization.h"
using namespace std;

int main(){
	Visualization* vs = Visualization::getInstance();
	vs->print_text<bool>("testKey", true);
	vs->visualize();
	return 0;
}
