#include "eLib/utils/EMath.hpp"
#include "eLib/eDriver/Sensors/EBumper.hpp"
#include "eLib/Configurator.hpp"
using namespace elib;

namespace elib{

    EBumperSwitch::EBumperSwitch(char port) : bumperSwitch(port){}
    bool EBumperSwitch::getStatus(){return bumperSwitch.get_value();}
}