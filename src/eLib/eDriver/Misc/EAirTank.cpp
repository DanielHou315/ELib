#include "eLib/eDriver/Misc/EAirTank.hpp"
#include "eLib/eDriver/Misc/EController.hpp"
#include "eLib/utils/EMath.hpp"
using namespace elib;

namespace elib{

    //--------------------------------
    // AirTank Functions
    //--------------------------------
    AirTank::AirTank() : airtankwatcher(std::bind(&AirTank::CheckPSI,this)){CurPSI = MaxPSIByLaw;}

    void AirTank::CheckPSI(){
        while(true){
            // Print PSI
            char* msg = (char*) malloc(16 * sizeof(char));
            if(CurPSI - floor(CurPSI) == 0) sprintf(msg, "PSI: %.0f", CurPSI);
            else sprintf(msg, "PSI: %.1f", CurPSI);
            EController::print(0, msg);
            // Warn Low Air
            if (CurPSI <= 50 && !lowAirWarned){
                EController::rumble(". .");
                lowAirWarned = true;
            }
            pros::delay(20);
        }
    }

    void AirTank::AddActuation(int num){
        CurPSI -= 2.5*num;
        if (CurPSI < 0) CurPSI = 0;
    }
    int AirTank::GetPSI(){return CurPSI;}


} // namespace elib
