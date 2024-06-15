/*
 * File: commands.h
 *
 * stlink commands
 */

#ifndef COMMANDS_H
#define COMMANDS_H

enum stlink_commands {
    STLINK_GET_VERSION                   = 0xF1,
    STLINK_DEBUG_COMMAND                 = 0xF2,
    STLINK_DFU_COMMAND                   = 0xF3,
    STLINK_GET_CURRENT_MODE              = 0xF5,
    STLINK_GET_TARGET_VOLTAGE            = 0xF7,
    STLINK_GET_VERSION_APIV3             = 0xFB
};

typedef enum type_command_e
{
    MBL_DBG_CMD_GDB = 0x27,
    PLACEHOLDER_CMD = 0xA5     
} type_command_e;

typedef enum debug_commands_e {

    DEBUG_FORCEDEBUG        = 0x02,
    DEBUG_READALLREGS       = 0x04,
    DEBUG_READREG           = 0x05,
    DEBUG_WRITEREG          = 0x06,
    DEBUG_READMEM_32BIT     = 0x07,
    DEBUG_WRITEMEM_32BIT    = 0x08,
    DEBUG_RUNCORE           = 0x09,
    DEBUG_STEPCORE          = 0x0a,
    DEBUG_PAUSE             = 0x10,
    

    DEBUG_PRINT_LOG         = 0x11,

    DEBUG_WRITEMEM_8BIT     = 0x0d,
    DEBUG_WRITEDEBUGREG     = 0x0f,
    DEBUG_EXIT              = 0x21,
    DEBUG_READCOREID        = 0x22,
    DEBUG_RESET             = 0x32,
    DEBUG_READDEBUGREG      = 0x36,

    DEBUG_START_TRACE_RX    = 0x40,
    DEBUG_STOP_TRACE_RX     = 0x41,
    DEBUG_GET_TRACE_NB      = 0x42,
    DEBUG_CHANGE_SETTINGS   = 0x43
} debug_commands_e;

#endif // COMMANDS_H
