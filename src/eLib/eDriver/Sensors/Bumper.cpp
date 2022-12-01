#include "eLib/utils/Math.hpp"
#include "eLib/eDriver/Sensors/Bumper.hpp"
#include "eLib/Configurator.hpp"
using namespace elib;

namespace elib{
    EBumperSwitch::EBumperSwitch(char port) : bumperSwitch(port){}
    bool EBumperSwitch::getStatus(){return bumperSwitch.get_value();}
}