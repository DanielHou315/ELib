#include "eLib/eDriver/Actuators/EPiston.hpp"
using namespace elib;

namespace elib{

    //----------------------------------------------
    //  E Pison Functions
    //----------------------------------------------
    EPiston::EPiston(const PistonConfig& piston_cfg, AirTank * AirtankPtr) : piston(piston_cfg.port, piston_cfg.initStatus)                {
        pistonStatus = piston_cfg.initStatus;
        pistonNumber = piston_cfg.number_of_pistons;                     // num for how many pistons connected to this one port
        actionType = piston_cfg.actionType;
        air_tank = AirtankPtr;
    }

    // Actuate Piston
    void EPiston::actuatePiston(){
        if (pistonStatus == E_PISTON_RETRACTED) pistonStatus = E_PISTON_EXTENDED;
        else pistonStatus = E_PISTON_RETRACTED;
        piston.set_value(pistonStatus);
        // If ( Airtank pointer is not empty AND ( Double Action Piston OR Single Action Extended ) ), add actuation
        if (air_tank != NULL && (actionType == E_PISTON_DOUBLE_ACTION || pistonStatus)) air_tank->AddActuation(pistonNumber);
        return;
    }
    void EPiston::setPistonStatus(const bool& pStatus){if (pistonStatus != pStatus) actuatePiston();}
    PistonStatus EPiston::getStatus(){return pistonStatus;}

}