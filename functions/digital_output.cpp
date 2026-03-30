#include "digital_output.h"

void Digital_Output::Update()
{
    if(!pConfig->bEnabled)
    {
        fVal = 0;
        palWriteLine(m_line, 0);
        return;
    }

    if(pConfig->nInput)
    {
        palWriteLine(m_line, 1);
        fVal = 1;
    }
    else
    {
        palWriteLine(m_line, 0);
        fVal = 0;
    }
}