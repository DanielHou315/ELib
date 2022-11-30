#ifndef E_AUTO_SELECTOR
#define E_AUTO_SELECTOR

namespace elib{

    class EAutoSelector{
    private:
        unsigned int maxAutoNumber, currentAutoNumber;
    public:
        EAutoSelector(unsigned int totalAutos);
        void changeAutoNumber();
        unsigned int getAutoNumber();
        unsigned int getTotalAutos();
    };

}


#endif