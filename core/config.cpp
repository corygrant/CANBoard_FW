#include "config.h"
#include "error.h"
#include "crc.h"
#include "param_registry.h"
#include "hal.h"

// Last 2KB sector of flash (sector 31, 0x0800F800)
#define CONFIG_SECTOR       31U
#define CONFIG_FLASH_OFFSET (CONFIG_SECTOR * 2048U)
#define CONFIG_FLASH        getBaseFlash(&EFLD1)

static void SetAllDefaultParams()
{
    for (uint16_t i = 0; i < NUM_PARAMS; i++)
        WriteParam(&stParams[i], stParams[i].nDefaultVal, false);
}

bool ReadConfig()
{
    CanboardConfig tempConfig;

    flash_error_t err = flashRead(CONFIG_FLASH, CONFIG_FLASH_OFFSET,
                                  sizeof(CanboardConfig), (uint8_t*)&tempConfig);
    if (err != FLASH_NO_ERROR)
        return false;

    if (tempConfig.stDevConfig.nConfigVersion != CONFIG_VERSION)
        return false;

    uint32_t storedCrc;
    err = flashRead(CONFIG_FLASH, CONFIG_FLASH_OFFSET + sizeof(CanboardConfig),
                    sizeof(uint32_t), (uint8_t*)&storedCrc);
    if (err != FLASH_NO_ERROR)
        return false;

    if (CalculateCRC32(&tempConfig, sizeof(CanboardConfig)) != storedCrc)
        return false;

    stConfig = tempConfig;
    return true;
}

bool WriteConfig()
{
    stConfig.stDevConfig.nConfigVersion = CONFIG_VERSION;

    flash_error_t err = flashStartEraseSector(CONFIG_FLASH, CONFIG_SECTOR);
    if (err != FLASH_NO_ERROR)
        return false;

    err = flashWaitErase(CONFIG_FLASH);
    if (err != FLASH_NO_ERROR)
        return false;

    err = flashProgram(CONFIG_FLASH, CONFIG_FLASH_OFFSET,
                       sizeof(CanboardConfig), (const uint8_t*)&stConfig);
    if (err != FLASH_NO_ERROR)
        return false;

    uint32_t crc = CalculateCRC32(&stConfig, sizeof(CanboardConfig));
    err = flashProgram(CONFIG_FLASH, CONFIG_FLASH_OFFSET + sizeof(CanboardConfig),
                       sizeof(uint32_t), (const uint8_t*)&crc);

    return (err == FLASH_NO_ERROR);
}

void InitConfig()
{
    eflStart(&EFLD1, NULL);

    if (!ReadConfig())
    {
        SetAllDefaultParams();
        if (!WriteConfig())
            Error::SetFatalError(FatalErrorType::ErrConfig, MsgSrc::Config);
    }
}
