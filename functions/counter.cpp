#include "counter.h"
#include "dbc.h"
#include "edge.h"

void Counter::Update()
{
    if (!pConfig->bEnabled)
    {
        fVal = 0;
        return;
    }

    // Reset
    if (Edge::Check(pConfig->eResetEdge, bLastReset, *pResetInput))
    {
        fVal = 0;
        return;
    }

    // Hold to reset
    if (pConfig->bHoldToReset)
    {
        if (*pIncInput && (SYS_TIME - nLastIncTime >= pConfig->nResetTime))
        {
            fVal = 0;
            return;
        }

        if (*pDecInput && (SYS_TIME - nLastDecTime >= pConfig->nResetTime))
        {
            fVal = 0;
            return;
        }
    }

    // Increment
    if (Edge::Check(pConfig->eIncEdge, bLastInc, *pIncInput))
    {
        fVal++;
        if (fVal > pConfig->nMaxCount)
        {
            fVal = pConfig->bWrapAround ? 0 : pConfig->nMaxCount;
        }

        nLastIncTime = SYS_TIME;
    }

    // Decrement
    if (Edge::Check(pConfig->eDecEdge, bLastDec, *pDecInput))
    {
        if(fVal == 0)
        {
            fVal = pConfig->bWrapAround ? pConfig->nMaxCount : pConfig->nMinCount;
        }
        else
        {
            fVal--;
        }

        nLastDecTime = SYS_TIME;
    }

    bLastInc = *pIncInput;
    bLastDec = *pDecInput;
    bLastReset = *pResetInput;
}