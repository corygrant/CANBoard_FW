#pragma once

#include <cstdint>

enum class FatalErrorType : uint8_t
{
    NoError = 0,
    ErrIWDG,
    ErrMailbox,
    ErrTask,
    ErrConfig,
    ErrFRAM,
    ErrADC,
    ErrTempSensor,
    ErrUSB,
    ErrCAN,
    ErrCRC,
    ErrI2C,
    ErrRCC,
    ErrTemp,
    ErrPwm,
    ErrVarMap
};

//=============================================================================
// Message Enums
//=============================================================================
enum class MsgCmd : uint8_t
{
    Null = 0,
    Read = 1,
    Write = 2,
    ReadParamNotFound = 5,

    ReadAll = 10,
    ReadAllRsp = 11,
    ReadAllComplete = 12,
    ReadAllModified = 13,

    WriteAll = 20,
    WriteAllVal = 21,
    WriteAllComplete = 22,
    WriteAllModified = 23,
    WriteAllParamNotFound = 25,
    WriteAllOutOfRange = 26,

    BurnSettings = 30,
    Version = 31,
    Sleep = 32,
    Bootloader = 33,
    CheckCrc = 34,
    CheckCrcRsp = 35,

    Invalid = 0xFF
};

enum class MsgType : uint8_t
{
    Info = 'F',
    Warning = 'R',
    Error = 'E'
};

enum class MsgSrc : uint8_t
{
    State_Run = 1,
    State_Sleep,
    State_Overtemp,
    State_Error,
    Overcurrent,
    Voltage,
    CANbus,
    USB,
    Overtemp,
    Config,
    FRAM,
    Analog,
    I2C,
    TempSensor,
    USBConnection,
    Init
};

//=============================================================================
// Logic Operators
//=============================================================================
enum class Operator : uint8_t
{
    Equal,
    NotEqual,
    GreaterThan,
    LessThan,
    GreaterThanOrEqual,
    LessThanOrEqual,
    BitwiseAnd,
    BitwiseNand
};

enum class BoolOperator : uint8_t
{
    And,
    Or,
    Nor
};

//=============================================================================
// CAN
//=============================================================================
enum class CanBitrate : uint8_t
{
    Bitrate_1000K,
    Bitrate_500K,
    Bitrate_250K,
    Bitrate_125K
};

//=============================================================================
// Input
//=============================================================================
enum class InputMode : uint8_t
{
    Momentary,
    Latching
};

enum class InputEdge : uint8_t
{
    Rising,
    Falling,
    Both
};

enum class InputPull : uint8_t
{
    None,
    Up,
    Down
};