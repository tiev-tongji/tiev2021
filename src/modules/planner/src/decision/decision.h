#ifndef __DECISION__H__
#define __DECISION__H__
#include "tiev_fsm.h"
namespace TiEV {

class MachineManager {
 public:
  MachineManager(const MachineManager&) = delete;
  MachineManager&        operator=(const MachineManager&) = delete;
  Context                context;
  FSM::Instance          machine{context};
  static MachineManager& getInstance() {
    static MachineManager instance;
    return instance;
  }

 private:
  MachineManager() {}
  ~MachineManager() {}
};
void runTiEVFSM();
void sendPath();
void requestGlobalPathFromMapServer();

}  // namespace TiEV

#endif  //!__DECISION__H__