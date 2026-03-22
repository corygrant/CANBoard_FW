#include "input.h"

bool Input::Check(InputMode eMode, bool bInvert, bool bVal)
{
    bVal = bVal ^ bInvert;

    if (bVal != bLast)
    {
        if (eMode == InputMode::Momentary)
            bOut = bVal;
        else if ((eMode == InputMode::Latching) && 
                 (bVal == true))
            bOut = !bOut;
    }

    bLast = bVal;

    return bOut;
}