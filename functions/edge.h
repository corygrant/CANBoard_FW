#pragma once

#include <cstdint>
#include "config.h"

class Edge
{
public:
    static bool Check(InputEdge eEdge, bool bPrev, bool bCurr)
    {
        switch (eEdge)
        {
        case InputEdge::Rising:
            return !bPrev && bCurr;
        case InputEdge::Falling:
            return bPrev && !bCurr;
        case InputEdge::Both:
            return bPrev != bCurr;
        default:
            return false;
        }
    }
};