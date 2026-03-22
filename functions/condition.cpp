#include "condition.h"
#include "dbc.h"

void Condition::Update()
{
    if (!pConfig->bEnabled)
    {
        fVal = 0;
        return;
    }

    switch(pConfig->eOperator)
    {
        case Operator::Equal:
            fVal = *pInput == pConfig->fArg;
            break;

        case Operator::NotEqual:
            fVal = *pInput != pConfig->fArg;
            break;

        case Operator::GreaterThan:
            fVal = *pInput > pConfig->fArg;
            break;

        case Operator::LessThan:
            fVal = *pInput < pConfig->fArg;
            break;

        case Operator::GreaterThanOrEqual:
            fVal = *pInput >= pConfig->fArg;
            break;

        case Operator::LessThanOrEqual:
            fVal = *pInput <= pConfig->fArg;
            break;

        case Operator::BitwiseAnd:
            fVal = (uint16_t)(*pInput) & (uint16_t)(pConfig->fArg);
            break;

        case Operator::BitwiseNand:
            fVal = ~((uint16_t)(*pInput) & (uint16_t)(pConfig->fArg));
            break;

        default:
            fVal = 0;
            break;
    }
}