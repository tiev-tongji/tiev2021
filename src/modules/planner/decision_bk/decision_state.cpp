#include "decision_state.h"

namespace TiEV{

/*a demo state implement*/
void StateDemo::entry(){
	cout << "Entry StateDemo State..." << endl;
}

void StateDemo::exit(){
	cout << "Exit StateDemo State..." << endl;
}

void StateDemo::react(PlanningEvent const & e){
	//what to do when planning
	transit<StateDemo>();
}

}//end TiEV
