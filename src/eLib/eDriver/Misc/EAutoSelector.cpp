#include "eLib/eDriver/Misc/EAutoSelector.hpp"
using namespace elib;

#include "pros/misc.hpp"

namespace elib{

    EAutoSelector::EAutoSelector(unsigned int totalAutos){
        maxAutoNumber = totalAutos;
        currentAutoNumber = 0;
    }
    void EAutoSelector::changeAutoNumber(){
        if(!pros::competition::is_autonomous()){
            currentAutoNumber++;
            if(currentAutoNumber >= maxAutoNumber) currentAutoNumber -= maxAutoNumber;
        }
    }
    unsigned int EAutoSelector::getAutoNumber(){return currentAutoNumber;}
    unsigned int EAutoSelector::getTotalAutos(){return maxAutoNumber;}

}