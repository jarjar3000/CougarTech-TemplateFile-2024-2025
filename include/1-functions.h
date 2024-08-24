using namespace vex;
#include "0-variables.h"
#include "robot-config.h"

void clamp()
{
    if (extended)
    {
        clamp1.set(true);
        clamp2.set(true);
        extended = false;
    }
    else 
    {
        clamp1.set(false);
        clamp2.set(false);
        extended = true; 
    }
}