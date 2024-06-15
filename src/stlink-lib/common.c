/* == nightwalker-87: TODO: CONTENT AND USE OF THIS SOURCE FILE IS TO BE VERIFIED (07.06.2023) == */
/* TODO: This file should be split up into new or existing modules. */

/*
 * File: common.c
 *
 *
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
// #include <sys/stat.h>  // TODO: Check use
// #include <sys/types.h> // TODO: Check use

#include <stlink.h>

#include "chipid.h"
#include "helper.h"
#include "logging.h"
#include "map_file.h"
#include "md5.h"
#include "register.h"
#include "uart.h"

#ifndef O_BINARY
#define O_BINARY 0
#endif

#ifdef _MSC_VER
#define __attribute__(x)
#endif




typedef bool (*save_block_fn)(void *arg, uint8_t *block, ssize_t len);

static void stop_wdg_in_debug(stlink_t *);
int32_t stlink_jtag_reset(stlink_t *, int32_t);
int32_t stlink_soft_reset(stlink_t *, int32_t);
void _parse_version(stlink_t *, stlink_version_t *);

// Functions below are defined in stlink.h (see line num before function)
// 252
void stlink_close(stlink_t *sl) {
  DLOG("*** stlink_close ***\n");

  if (!sl) {
    return;
  }

  sl->backend->close(sl);
  free(sl);
}

// 250
int32_t stlink_exit_debug_mode(stlink_t *sl) {
  DLOG("*** stlink_exit_debug_mode ***\n");

  if (sl->flash_type != STM32_FLASH_TYPE_UNKNOWN &&
      sl->core_stat != TARGET_RESET) {
    // stop debugging if the target has been identified
    stlink_write_debug32(sl, STLINK_REG_DHCSR, STLINK_REG_DHCSR_DBGKEY);
  }

  return (sl->backend->exit_debug_mode(sl));
}


static void stop_wdg_in_debug(stlink_t *sl) {
  uint32_t dbgmcu_cr;
  uint32_t set;
  uint32_t value;

  switch (sl->flash_type) {
  case STM32_FLASH_TYPE_F0_F1_F3:
  case STM32_FLASH_TYPE_F1_XL:
  case STM32_FLASH_TYPE_G4:
    dbgmcu_cr = STM32F0_DBGMCU_CR;
    set = (1 << STM32F0_DBGMCU_CR_IWDG_STOP) |
          (1 << STM32F0_DBGMCU_CR_WWDG_STOP);
    break;
  case STM32_FLASH_TYPE_F2_F4:
  case STM32_FLASH_TYPE_F7:
  case STM32_FLASH_TYPE_L4:
    dbgmcu_cr = STM32F4_DBGMCU_APB1FZR1;
    set = (1 << STM32F4_DBGMCU_APB1FZR1_IWDG_STOP) |
          (1 << STM32F4_DBGMCU_APB1FZR1_WWDG_STOP);
    break;
  case STM32_FLASH_TYPE_L0_L1:
  case STM32_FLASH_TYPE_H7:
    dbgmcu_cr = STM32H7_DBGMCU_APB1HFZ;
    set = (1 << STM32H7_DBGMCU_APB1HFZ_IWDG_STOP);
    break;
  case STM32_FLASH_TYPE_WB_WL:
    dbgmcu_cr = STM32WB_DBGMCU_APB1FZR1;
    set = (1 << STM32WB_DBGMCU_APB1FZR1_IWDG_STOP) |
          (1 << STM32WB_DBGMCU_APB1FZR1_WWDG_STOP);
    break;
  default:
    return;
  }

  if (!stlink_read_debug32(sl, dbgmcu_cr, &value)) {
    stlink_write_debug32(sl, dbgmcu_cr, value | set);
  }
}
// 271
// Force the core into the debug mode -> halted state.
int32_t stlink_force_debug(stlink_t *sl) {
  DLOG("*** stlink_force_debug_mode ***\n");
  int32_t res = sl->backend->force_debug(sl);
  if (res) {
     return (res);
  }

  // Stop the watchdogs in the halted state for suppress target reboot
  stop_wdg_in_debug(sl);
  sl->core_stat = TARGET_HALTED;
  return (0);
}

// 251
int32_t stlink_exit_dfu_mode(stlink_t *sl) {
  DLOG("*** stlink_exit_dfu_mode ***\n");
  return (sl->backend->exit_dfu_mode(sl));
}

// 253
int32_t stlink_core_id(stlink_t *sl) {
  int32_t ret;

  DLOG("*** stlink_core_id ***\n");
  ret = sl->backend->core_id(sl);

  if (ret == -1) {
    ELOG("Failed to read core_id\n");
    return (ret);
  }

  if (sl->verbose > 2) {
    stlink_print_data(sl);
  }

  DLOG("core_id = 0x%08x\n", sl->core_id);
  return (ret);
}

// 287
// stlink_chip_id() is called by stlink_load_device_params()
// do not call this procedure directly.
int32_t stlink_chip_id(stlink_t *sl, uint32_t *chip_id) {
  int32_t ret;
  cortex_m3_cpuid_t cpu_id;

  // Read the CPU ID to determine where to read the core id
  if (stlink_cpu_id(sl, &cpu_id) ||
      cpu_id.implementer_id != STLINK_REG_CMx_CPUID_IMPL_ARM) {
    ELOG("Can not connect to target. Please use \'connect under reset\' and try again\n");
    return -1;
  }

  /*
   * the chip_id register in the reference manual have
   * DBGMCU_IDCODE / DBG_IDCODE name
   */

  if ((sl->core_id == STM32_CORE_ID_M7F_M33_SWD || sl->core_id == STM32_CORE_ID_M7F_M33_JTAG) &&
      cpu_id.part == STLINK_REG_CMx_CPUID_PARTNO_CM7) {
    // STM32H7 chipid in 0x5c001000 (RM0433 pg3189)
    ret = stlink_read_debug32(sl, 0x5c001000, chip_id);
  } else if (cpu_id.part == STLINK_REG_CMx_CPUID_PARTNO_CM0 ||
             cpu_id.part == STLINK_REG_CMx_CPUID_PARTNO_CM0P) {
    // STM32F0 (RM0091, pg914; RM0360, pg713)
    // STM32L0 (RM0377, pg813; RM0367, pg915; RM0376, pg917)
    // STM32G0 (RM0444, pg1367)
    ret = stlink_read_debug32(sl, 0x40015800, chip_id);
  } else if (cpu_id.part == STLINK_REG_CMx_CPUID_PARTNO_CM33) {
    // STM32L5 (RM0438, pg2157)
    ret = stlink_read_debug32(sl, 0xE0044000, chip_id);
  } else /* СM3, СM4, CM7 */ {
    // default chipid address

    // STM32F1 (RM0008, pg1087; RM0041, pg681)
    // STM32F2 (RM0033, pg1326)
    // STM32F3 (RM0316, pg1095; RM0313, pg874)
    // STM32F7 (RM0385, pg1676; RM0410, pg1912)
    // STM32L1 (RM0038, pg861)
    // STM32L4 (RM0351, pg1840; RM0394, pg1560)
    // STM32G4 (RM0440, pg2086)
    // STM32WB (RM0434, pg1406)
    ret = stlink_read_debug32(sl, 0xE0042000, chip_id);
  }

  if (ret || !(*chip_id)) {
    *chip_id = 0;
    ret = ret?ret:-1;
    ELOG("Could not find chip id!\n");
  } else {
    *chip_id = (*chip_id) & 0xfff;

    // Fix chip_id for F4 rev A errata, read CPU ID, as CoreID is the same for
    // F2/F4
    if (*chip_id == 0x411 && cpu_id.part == STLINK_REG_CMx_CPUID_PARTNO_CM4) {
      *chip_id = 0x413;
    }
  }

  return (ret);
}

// 288
/**
 * Cortex M tech ref manual, CPUID register description
 * @param sl stlink context
 * @param cpuid pointer to the result object
 */
int32_t stlink_cpu_id(stlink_t *sl, cortex_m3_cpuid_t *cpuid) {
  uint32_t raw;

  if (stlink_read_debug32(sl, STLINK_REG_CM3_CPUID, &raw)) {
    cpuid->implementer_id = 0;
    cpuid->variant = 0;
    cpuid->part = 0;
    cpuid->revision = 0;
    return (-1);
  }

  cpuid->implementer_id = (raw >> 24) & 0x7f;
  cpuid->variant = (raw >> 20) & 0xf;
  cpuid->part = (raw >> 4) & 0xfff;
  cpuid->revision = raw & 0xf;
  return (0);
}

// 303
/**
 * Reads and decodes the flash parameters, as dynamically as possible
 * @param sl
 * @return 0 for success, or -1 for unsupported core type.
 */
int32_t stlink_load_device_params(stlink_t *sl) {
  // This seems to normally work so is unnecessary info for a normal user.
  // Demoted to debug. -- REW
  DLOG("Loading device parameters....\n");
  const struct stlink_chipid_params *params = NULL;
  stlink_core_id(sl);
  uint32_t flash_size;

  if (stlink_chip_id(sl, &sl->chip_id)) {
    return (-1);
  }

  params = stlink_chipid_get_params(sl->chip_id);

  if (params == NULL) {
    WLOG("unknown chip id! %#x\n", sl->chip_id);
    return (-1);
  }

  if (params->flash_type == STM32_FLASH_TYPE_UNKNOWN) {
    WLOG("Invalid flash type, please check device declaration\n");
    sl->flash_size = 0;
    return (0);
  }

  // These are fixed...
  sl->flash_base = STM32_FLASH_BASE;
  sl->sram_base = STM32_SRAM_BASE;
  stlink_read_debug32(sl, (params->flash_size_reg) & ~3, &flash_size);

  if (params->flash_size_reg & 2) {
    flash_size = flash_size >> 16;
  }

  flash_size = flash_size & 0xffff;

  if ((sl->chip_id == STM32_CHIPID_L1_MD ||
       sl->chip_id == STM32_CHIPID_F1_VL_MD_LD ||
       sl->chip_id == STM32_CHIPID_L1_MD_PLUS) &&
      (flash_size == 0)) {
    sl->flash_size = 128 * 1024;
  } else if (sl->chip_id == STM32_CHIPID_L1_CAT2) {
    sl->flash_size = (flash_size & 0xff) * 1024;
  } else if ((sl->chip_id & 0xFFF) == STM32_CHIPID_L1_MD_PLUS_HD) {
    // 0 is 384k and 1 is 256k
    if (flash_size == 0) {
      sl->flash_size = 384 * 1024;
    } else {
      sl->flash_size = 256 * 1024;
    }
  } else {
    sl->flash_size = flash_size * 1024;
  }

  sl->flash_type = params->flash_type;
  sl->flash_pgsz = params->flash_pagesize;
  sl->sram_size = params->sram_size;
  sl->sys_base = params->bootrom_base;
  sl->sys_size = params->bootrom_size;
  sl->option_base = params->option_base;
  sl->option_size = params->option_size;
  sl->chip_flags = params->flags;

  // medium and low devices have the same chipid. ram size depends on flash
  // size. STM32F100xx datasheet Doc ID 16455 Table 2
  if (sl->chip_id == STM32_CHIPID_F1_VL_MD_LD && sl->flash_size < 64 * 1024) {
    sl->sram_size = 0x1000;
  }

  if (sl->chip_id == STM32_CHIPID_G4_CAT3 ||
      sl->chip_id == STM32_CHIPID_G4_CAT4) {
    uint32_t flash_optr;
    stlink_read_debug32(sl, FLASH_Gx_OPTR, &flash_optr);

    if (!(flash_optr & (1 << FLASH_G4_OPTR_DBANK))) {
      sl->flash_pgsz <<= 1;
    }
  }

  if (sl->chip_id == STM32_CHIPID_L5x2xx) {
    uint32_t flash_optr;
    stlink_read_debug32(sl, FLASH_L5_OPTR, &flash_optr);

    if (sl->flash_size == 512*1024 && (flash_optr & (1 << 22)) != 0) {
      sl->flash_pgsz = 0x800;
    }
  }
  
  // H7 devices with small flash has one bank
  if (sl->chip_flags & CHIP_F_HAS_DUAL_BANK &&
      sl->flash_type == STM32_FLASH_TYPE_H7) {
    if ((sl->flash_size / sl->flash_pgsz) <= 1)
      sl->chip_flags &= ~CHIP_F_HAS_DUAL_BANK;
  }

  ILOG("%s: %u KiB SRAM, %u KiB flash in at least %u %s pages.\n",
      params->dev_type, (sl->sram_size / 1024), (sl->flash_size / 1024),
      (sl->flash_pgsz < 1024) ? sl->flash_pgsz : (sl->flash_pgsz / 1024),
      (sl->flash_pgsz < 1024) ? "byte" : "KiB");

  return (0);
}

// 254
int32_t stlink_reset(stlink_t *sl, enum reset_type type) {
  uint32_t dhcsr;
  uint32_t timeout;

  DLOG("*** stlink_reset ***\n");

  sl->core_stat = TARGET_RESET;

  if (type == RESET_AUTO) {
    // clear S_RESET_ST in DHCSR register for reset state detection
    stlink_read_debug32(sl, STLINK_REG_DHCSR, &dhcsr);
  }

  if (type == RESET_AUTO) {
    sl->backend->reset(sl);
    usleep(10000);
  }
  uint32_t timeout2 = time_ms() + 1000;
  while(time_ms() < timeout2);

  if (open_uart(sl))
  {
      ELOG("Reset openUart failed.\n");
      return -1;
  };
  if (sl->backend->change_settings(sl))
  {
      ELOG("Reset changeSettings failed.\n");
      return -1;
  };
  if (reopen_uart(sl))
  {
      ELOG("Reset reopen failed.\n");
      return -1;
  };
  return 0;
}

int32_t stlink_soft_reset(stlink_t *sl, int32_t halt_on_reset) {
  return (0);
}

// 255
int32_t stlink_run(stlink_t *sl, enum run_type type) {
  struct stlink_reg rr;
  DLOG("*** stlink_run ***\n");

  /* Make sure we are in Thumb mode
   * Cortex-M chips don't support ARM mode instructions
   * xPSR may be incorrect if the vector table has invalid data */
  stlink_read_reg(sl, 16, &rr);
  if ((rr.xpsr & (1 << 24)) == 0) {
    ILOG("Go to Thumb mode\n");
    stlink_write_reg(sl, rr.xpsr | (1 << 24), 16);
  }

  return (sl->backend->run(sl, type));
}


// 293
// this function is called by stlink_status()
// do not call stlink_core_stat() directly, always use stlink_status()
void stlink_core_stat(stlink_t *sl) {
  switch (sl->core_stat) {
  case TARGET_RUNNING:
    DLOG("  core status: running\n");
    return;
  case TARGET_HALTED:
    DLOG("  core status: halted\n");
    return;
  case TARGET_RESET:
    DLOG("  core status: reset\n");
    return;
  case TARGET_DEBUG_RUNNING:
    DLOG("  core status: debug running\n");
    return;
  default:
    DLOG("  core status: unknown\n");
  }
}

// 256
int32_t stlink_status(stlink_t *sl) {
  int32_t ret;

  DLOG("*** stlink_status ***\n");
  ret = sl->backend->status(sl);
  stlink_core_stat(sl);
  return (ret);
}



// 299
bool stlink_is_core_halted(stlink_t *sl) {
  stlink_status(sl);
  return (sl->core_stat == TARGET_HALTED);
}

// 269
int32_t stlink_step(stlink_t *sl) {
  DLOG("*** stlink_step ***\n");
  return (sl->backend->step(sl));
}

// 274
int32_t stlink_trace_enable(stlink_t *sl, uint32_t frequency) {
  DLOG("*** stlink_trace_enable ***\n");
  return (sl->backend->trace_enable(sl, frequency));
}

// 275
int32_t stlink_trace_disable(stlink_t *sl) {
  DLOG("*** stlink_trace_disable ***\n");
  return (sl->backend->trace_disable(sl));
}

// 276
int32_t stlink_trace_read(stlink_t *sl, uint8_t *buf, uint32_t size) {
  return (sl->backend->trace_read(sl, buf, size));
}

// 294
void stlink_print_data(stlink_t *sl) {
  if (sl->q_len <= 0 || sl->verbose < UDEBUG) {
    return;
  }

  if (sl->verbose > 2) {
    DLOG("data_len = %d 0x%x\n", sl->q_len, sl->q_len);
  }

  for (int32_t i = 0; i < sl->q_len; i++) {
    if (i % 16 == 0) {
      /*
      if (sl->q_data_dir == Q_DATA_OUT) {
          fprintf(stdout, "\n<- 0x%08x ", sl->q_addr + i);
      } else {
          fprintf(stdout, "\n-> 0x%08x ", sl->q_addr + i);
      }
      */
    }
    // DLOG(" %02x", (uint32_t) sl->q_buf[i]);
    fprintf(stderr, " %02x", (uint32_t)sl->q_buf[i]);
  }
  // DLOG("\n\n");
  fprintf(stderr, "\n");
}


int32_t stlink_jtag_reset(stlink_t *sl, int32_t value) {
  DLOG("*** stlink_jtag_reset %d ***\n", value);
  return (sl->backend->jtag_reset(sl, value));
}
