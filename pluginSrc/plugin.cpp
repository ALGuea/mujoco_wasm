#include <mujoco/mjplugin.h>
#include "Pid.h"

namespace mujoco::plugin::actuator {

mjPLUGIN_LIB_INIT { Pid::RegisterPlugin(); }

}  // namespace mujoco::plugin::actuator