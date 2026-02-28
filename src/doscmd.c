/* sd2iec - SD/MMC to Commodore serial bus interface/controller
   Copyright (C) 2007-2017  Ingo Korb <ingo@akana.de>

   Inspired by MMC2IEC by Lars Pontoppidan et al.

   FAT filesystem access based on code from ChaN and Jim Brain, see ff.c|h.

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; version 2 of the License only.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA


   doscmd.c: Command channel parser

*/

#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "crc.h"
#include "d64ops.h"
#include "dirent.h"
#include "diskchange.h"
#include "diskio.h"
#include "display.h"
#include "eeprom-conf.h"
#include "errormsg.h"
#include "fastloader.h"
#include "fastloader-ll.h"
#include "fatops.h"
#include "ff.h"
#include "fileops.h"
#include "filesystem.h"
#include "flags.h"
#include "iec.h"
#include "led.h"
#include "parser.h"
#include "system.h"
#include "time.h"
// #include "rtc.h"
#include "uart.h"
#include "ustring.h"
#include "utils.h"
#include "wrapops.h"
#include "doscmd.h"

#define CURSOR_RIGHT 0x1d

/* ---- Fastloader tables ---- */

enum {
  RXTX_NONE,

  RXTX_GEOS_1MHZ,
  RXTX_GEOS_2MHZ,
  RXTX_GEOS_1581_21,
  RXTX_WHEELS_1MHZ,
  RXTX_WHEELS_2MHZ,
  RXTX_WHEELS44_1541,
  RXTX_WHEELS44_1581,

  RXTX_FC3OF_PAL,
  RXTX_FC3OF_NTSC,
};

typedef uint8_t (*fastloader_rx_t)(void);
typedef uint8_t (*fastloader_tx_t)(uint8_t byte);
typedef void    (*fastloader_handler_t)(uint8_t param);

struct fastloader_rxtx_s {
  fastloader_rx_t rxfunc;
  fastloader_tx_t txfunc;
};

static const PROGMEM struct fastloader_rxtx_s fl_rxtx_table[] = {
#ifdef CONFIG_LOADER_GEOS
  [RXTX_GEOS_1MHZ]     = { geos_get_byte_1mhz,     geos_send_byte_1mhz     },
  [RXTX_GEOS_2MHZ]     = { geos_get_byte_2mhz,     geos_send_byte_2mhz     },
  [RXTX_GEOS_1581_21]  = { geos_get_byte_2mhz,     geos_send_byte_1581_21  },
# ifdef CONFIG_LOADER_WHEELS
  [RXTX_WHEELS_1MHZ]   = { wheels_get_byte_1mhz,   wheels_send_byte_1mhz   },
  [RXTX_WHEELS_2MHZ]   = { geos_get_byte_2mhz,     geos_send_byte_1581_21  },
  [RXTX_WHEELS44_1541] = { wheels44_get_byte_1mhz, wheels_send_byte_1mhz   },
  [RXTX_WHEELS44_1581] = { wheels44_get_byte_2mhz, wheels44_send_byte_2mhz },
# endif
#endif
#ifdef CONFIG_LOADER_FC3
  [RXTX_FC3OF_PAL]     = { NULL, fc3_oldfreeze_pal_send  },
  [RXTX_FC3OF_NTSC]    = { NULL, fc3_oldfreeze_ntsc_send },
#endif
};

struct fastloader_crc_s {
  uint16_t crc;
  uint8_t  loadertype;
  uint8_t  rxtx;
};

static const PROGMEM struct fastloader_crc_s fl_crc_table[] = {
#ifdef CONFIG_LOADER_TURBODISK
  { 0x9c9f, FL_TURBODISK,        RXTX_NONE          },
#endif
#ifdef CONFIG_LOADER_FC3
  { 0xdab0, FL_FC3_LOAD,         RXTX_NONE          }, // Final Cartridge III
  { 0x973b, FL_FC3_LOAD,         RXTX_NONE          }, // Final Cartridge III variation
  { 0x7e38, FL_FC3_LOAD,         RXTX_NONE          }, // EXOS v3
  { 0x1b30, FL_FC3_SAVE,         RXTX_NONE          }, // note: really early CRC; lots of C64 code at the end
  { 0x8b0e, FL_FC3_SAVE,         RXTX_NONE          }, // variation
  { 0x9930, FL_FC3_FREEZED,      RXTX_NONE          },
  { 0x0281, FL_FC3_OLDFREEZED,   RXTX_FC3OF_PAL     }, // older freezed-file loader, PAL
  { 0xc196, FL_FC3_OLDFREEZED,   RXTX_FC3OF_NTSC    }, // older freezed-file loader, NTSC
#endif
#ifdef CONFIG_LOADER_DREAMLOAD
  { 0x2e69, FL_DREAMLOAD,        RXTX_NONE          },
#endif
#ifdef CONFIG_LOADER_ULOAD3
  { 0xdd81, FL_ULOAD3,           RXTX_NONE          },
#endif
#ifdef CONFIG_LOADER_ELOAD1
  { 0x393e, FL_ELOAD1,           RXTX_NONE          },
#endif
#ifdef CONFIG_LOADER_EPYXCART
  { 0x5a01, FL_EPYXCART,         RXTX_NONE          },
#endif
#ifdef CONFIG_LOADER_GEOS
  { 0xb979, FL_GEOS_S1_64,       RXTX_GEOS_1MHZ     }, // GEOS 64 stage 1
  { 0x2469, FL_GEOS_S1_128,      RXTX_GEOS_1MHZ     }, // GEOS 128 stage 1
  { 0x4d79, FL_GEOS_S23_1541,    RXTX_GEOS_1MHZ     }, // GEOS 64 1541 stage 2
  { 0xb2bc, FL_GEOS_S23_1541,    RXTX_GEOS_1MHZ     }, // GEOS 128 1541 stage 2
  { 0xb272, FL_GEOS_S23_1541,    RXTX_GEOS_1MHZ     }, // GEOS 64/128 1541 stage 3 (Configure)
  { 0xdaed, FL_GEOS_S23_1571,    RXTX_GEOS_2MHZ     }, // GEOS 64/128 1571 stage 3 (Configure)
  { 0x3f8d, FL_GEOS_S23_1581,    RXTX_GEOS_2MHZ     }, // GEOS 64/128 1581 Configure 2.0
  { 0xc947, FL_GEOS_S23_1581,    RXTX_GEOS_1581_21  }, // GEOS 64/128 1581 Configure 2.1
# ifdef CONFIG_LOADER_WHEELS
  { 0xf140, FL_WHEELS_S1_64,     RXTX_WHEELS_1MHZ   }, // Wheels 64 stage 1
  { 0x737e, FL_WHEELS_S1_128,    RXTX_WHEELS_1MHZ   }, // Wheels 128 stage 1
  { 0x755a, FL_WHEELS_S2,        RXTX_WHEELS_1MHZ   }, // Wheels 64 1541 stage 2
  { 0x2920, FL_WHEELS_S2,        RXTX_WHEELS_1MHZ   }, // Wheels 128 1541 stage 2
  { 0x18e9, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 64 1571
  { 0x9804, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 64 1581
  { 0x48f5, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 64 FD native partition
  { 0x1356, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 64 FD emulation partition
  { 0xe885, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 64 HD native partition
  { 0x4eca, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 64 HD emulation partition
  { 0xdbf6, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 128 1571
  { 0xe4ab, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 128 1581
  { 0x6de5, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 128 FD native
  { 0x30ff, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 128 FD emulation
  { 0x46e7, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 128 HD native
  { 0x2253, FL_WHEELS_S2,        RXTX_WHEELS_2MHZ   }, // Wheels 128 HD emulation
  { 0xc26a, FL_WHEELS44_S2,      RXTX_WHEELS44_1541 }, // Wheels 64/128 4.4 1541
  { 0x550c, FL_WHEELS44_S2,      RXTX_WHEELS44_1541 }, // Wheels 64/128 4.4 1571
  { 0x825b, FL_WHEELS44_S2_1581, RXTX_WHEELS44_1581 }, // Wheels 64/128 4.4 1581
  { 0x245b, FL_WHEELS44_S2_1581, RXTX_WHEELS44_1581 }, // Wheels 64/128 4.4 1581
  { 0x7021, FL_WHEELS44_S2_1581, RXTX_WHEELS44_1581 }, // Wheels 64/128 4.4 1581
  { 0xd537, FL_WHEELS44_S2_1581, RXTX_WHEELS44_1581 }, // Wheels 64/128 4.4 1581
  { 0xf635, FL_WHEELS44_S2_1581, RXTX_WHEELS44_1581 }, // Wheels 64/128 4.4 1581
# endif
#endif
#ifdef CONFIG_LOADER_NIPPON
  { 0x43c1, FL_NIPPON,           RXTX_NONE          }, // Nippon
#endif
#ifdef CONFIG_LOADER_AR6
  { 0x4870, FL_AR6_1581_LOAD,    RXTX_NONE          },
  { 0x2925, FL_AR6_1581_SAVE,    RXTX_NONE          },
#endif
#ifdef CONFIG_LOADER_MMZAK
  { 0x12a6, FL_MMZAK,            RXTX_NONE          }, // Maniac Mansion/Zak McKracken
#endif
#ifdef CONFIG_LOADER_GIJOE
  { 0x0c92, FL_GI_JOE,           RXTX_NONE          }, // hacked-up GI Joe loader seen in an Eidolon crack
#endif
#ifdef CONFIG_LOADER_N0SDOS
  { 0x327d, FL_N0SDOS_FILEREAD,  RXTX_NONE          }, // CRC up to 0x65f to avoid junk data
#endif
#ifdef CONFIG_LOADER_SAMSJOURNEY
  { 0x6af4, FL_SAMSJOURNEY,      RXTX_NONE          }, // CRC of penultimate M-W
#endif

  { 0, FL_NONE, 0 }, // end marker
};

struct fastloader_handler_s {
  uint16_t             address;
  uint8_t              loadertype;
  fastloader_handler_t handler;
  uint8_t              parameter;
};

static const PROGMEM struct fastloader_handler_s fl_handler_table[] = {
#ifdef CONFIG_LOADER_TURBODISK
  { 0x0303, FL_TURBODISK,        load_turbodisk, 0 },
#endif
#ifdef CONFIG_LOADER_FC3
  { 0x059a, FL_FC3_LOAD,         load_fc3,       0 }, // FC3
  { 0x0400, FL_FC3_LOAD,         load_fc3,       0 }, // EXOS
  { 0x059c, FL_FC3_SAVE,         save_fc3,       0 },
  { 0x059a, FL_FC3_SAVE,         save_fc3,       0 }, // variation
  { 0x0403, FL_FC3_FREEZED,      load_fc3,       1 },
  { 0x057f, FL_FC3_OLDFREEZED,   load_fc3oldfreeze, 0},

#endif
#ifdef CONFIG_LOADER_DREAMLOAD
  { 0x0700, FL_DREAMLOAD,        load_dreamload, 0 },
#endif
#ifdef CONFIG_LOADER_ULOAD3
  { 0x0336, FL_ULOAD3,           load_uload3,    0 },
#endif
#ifdef CONFIG_LOADER_ELOAD1
  { 0x0300, FL_ELOAD1,           load_eload1,    0 },
#endif
#ifdef CONFIG_LOADER_GIJOE
  { 0x0500, FL_GI_JOE,           load_gijoe,     0 },
#endif
#ifdef CONFIG_LOADER_EPYXCART
  { 0x01a9, FL_EPYXCART,         load_epyxcart,  0 },
#endif
#ifdef CONFIG_LOADER_GEOS
  { 0x0457, FL_GEOS_S1_64,       load_geos_s1,   0 },
  { 0x0470, FL_GEOS_S1_128,      load_geos_s1,   1 },
  { 0x03e2, FL_GEOS_S23_1541,    load_geos,      0 },
  { 0x03dc, FL_GEOS_S23_1541,    load_geos,      0 },
  { 0x03ff, FL_GEOS_S23_1571,    load_geos,      0 },
  { 0x040f, FL_GEOS_S23_1581,    load_geos,      0 },
# ifdef CONFIG_LOADER_WHEELS
  { 0x0400, FL_WHEELS_S1_64,     load_wheels_s1, 0 },
  { 0x0400, FL_WHEELS_S1_128,    load_wheels_s1, 1 },
  { 0x0300, FL_WHEELS_S2,        load_wheels_s2, 0 },
  { 0x0400, FL_WHEELS44_S2,      load_wheels_s2, 0 },
  { 0x0300, FL_WHEELS44_S2_1581, load_wheels_s2, 0 },
  { 0x0500, FL_WHEELS44_S2_1581, load_wheels_s2, 0 },
# endif
#endif
#ifdef CONFIG_LOADER_NIPPON
  { 0x0300, FL_NIPPON,           load_nippon,    0 },
#endif
#ifdef CONFIG_LOADER_AR6
  { 0x0500, FL_AR6_1581_LOAD,    load_ar6_1581,  0 },
  { 0x05f4, FL_AR6_1581_SAVE,    save_ar6_1581,  0 },
#endif
#ifdef CONFIG_LOADER_MMZAK
  { 0x0500, FL_MMZAK,            load_mmzak,     0 },
#endif
#ifdef CONFIG_LOADER_N0SDOS
  { 0x041b, FL_N0SDOS_FILEREAD,  load_n0sdos_fileread, 0 },
#endif
#ifdef CONFIG_LOADER_SAMSJOURNEY
  { 0x0400, FL_SAMSJOURNEY,      load_samsjourney, 0 },
#endif

  { 0, FL_NONE, NULL, 0 }, // end marker
};
/* System partition G-P answer */
static const PROGMEM uint8_t system_partition_info[] = {
  0xff,0xe2,0x00,0x53,0x59,0x53,0x54,0x45,
  0x4d,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,0xa0,
  0xa0,0xa0,0xa0,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0d
};

#ifndef globalflags
/* AVR uses GPIOR for this */
uint8_t globalflags;
#endif

uint8_t command_buffer[CONFIG_COMMAND_BUFFER_SIZE+2];
uint8_t command_length,original_length;

date_t date_match_start;
date_t date_match_end;

uint16_t datacrc = 0xffff;
static fastloaderid_t previous_loader;

#ifdef CONFIG_STACK_TRACKING
//FIXME: AVR-only code
uint16_t minstack = RAMEND;

void __cyg_profile_func_enter (void *this_fn, void *call_site) __attribute__((no_instrument_function));
void __cyg_profile_func_exit  (void *this_fn, void *call_site) __attribute__((alias("__cyg_profile_func_enter")));

void __cyg_profile_func_enter (void *this_fn, void *call_site) {
  if (SP < minstack) minstack = SP;
}
#endif

#ifdef HAVE_RTC
/* Days of the week as used by the CMD FD */
static const PROGMEM uint8_t downames[] = "SUN.MON.TUESWED.THURFRI.SAT.";

/* Skeleton of the ASCII time format */
static const PROGMEM uint8_t asciitime_skel[] = " xx/xx/xx xx:xx:xx xM\r";
#endif


/* ------------------------------------------------------------------------- */
/*  Drive code exec helpers                                                  */
/* ------------------------------------------------------------------------- */

#ifdef CONFIG_CAPTURE_LOADERS
static uint8_t loader_buffer[CONFIG_CAPTURE_BUFFER_SIZE];
static uint8_t *loader_ptr = loader_buffer;
static uint8_t capture_count = 0;

/* Convert byte to two-character hex string */
static uint8_t *byte_to_hex(uint8_t num, uint8_t *str) {
  uint8_t tmp;

  tmp = (num & 0xf0) >> 4;
  if (tmp < 10)
    *str++ = '0' + tmp;
  else
    *str++ = 'a' + tmp - 10;

  tmp = num & 0x0f;
  if (tmp < 10)
    *str++ = '0' + tmp;
  else
    *str++ = 'a' + tmp - 10;

  return str;
}

/* Copy command to capture buffer */
static void dump_command(void) {
  if (loader_ptr - loader_buffer + command_length + 2+1 < CONFIG_CAPTURE_BUFFER_SIZE) {
    *loader_ptr++ = 'C';
    *loader_ptr++ = command_length;
    memcpy(loader_ptr, command_buffer, command_length);
    loader_ptr += command_length;
  } else {
    *loader_ptr = 'X';
  }
}

/* Dump buffer state */
static void dump_buffer_state(void) {
  if (CONFIG_CAPTURE_BUFFER_SIZE - (loader_ptr - loader_buffer) > sizeof(buffer_t)*CONFIG_BUFFER_COUNT+2) {
    *loader_ptr++ = 'B';
    *loader_ptr++ = sizeof(buffer_t);
    *loader_ptr++ = CONFIG_BUFFER_COUNT;
    memcpy(loader_ptr, buffers, sizeof(buffer_t) * CONFIG_BUFFER_COUNT);
    loader_ptr += sizeof(buffer_t) * CONFIG_BUFFER_COUNT;
  }
}

/* Save capture buffer to disk */
static void save_capbuffer(void) {
  uint8_t *ptr;
  FIL datafile;

  /* Use CRC and counter as file name */
  ptr = byte_to_hex(datacrc >> 8, ops_scratch);
  ptr = byte_to_hex(datacrc & 0xff, ptr);
  *ptr++ = '-';
  ptr = byte_to_hex(capture_count++, ptr);
  *ptr++ = '.';
  *ptr++ = 'd';
  *ptr++ = 'm';
  *ptr++ = 'p';
  *ptr   = 0;

  FRESULT res = f_open(&partition[0].fatfs, &datafile, ops_scratch, FA_WRITE | FA_CREATE_ALWAYS);
  if (res == FR_OK) {
    UINT byteswritten;

    /* Skip the error checks here */
    f_write(&datafile, loader_buffer, loader_ptr - loader_buffer, &byteswritten);
    f_close(&datafile);
  }

  /* Reset capture buffer */
  loader_ptr = loader_buffer;

  /* Notify the user */
  set_error(ERROR_DRIVE_NOT_READY);
}
#endif // CONFIG_CAPTURE_LOADERS

static void run_loader(uint16_t address) {

  if (detected_loader == FL_NONE)
    detected_loader = previous_loader;

#ifdef CONFIG_CAPTURE_LOADERS
  if (detected_loader == FL_NONE && datacrc != 0xffff) {
    dump_command();
    dump_buffer_state();
    save_capbuffer();
  }
#endif

  /* Try to find a handler for loader */
  const struct fastloader_handler_s *ptr = fl_handler_table;
  uint8_t loader,parameter;
  fastloader_handler_t handler;

  while ( (loader = pgm_read_byte(&ptr->loadertype)) != FL_NONE ) {
    if (detected_loader == loader &&
        address == pgm_read_word(&ptr->address)) {
      /* Found it */
      handler   = (fastloader_handler_t)pgm_read_word(&ptr->handler);
      parameter = pgm_read_byte(&ptr->parameter);

      /* Call */
      handler(parameter);

      break;
    }
    ptr++;
  }

  if (loader == FL_NONE)
    set_error_ts(ERROR_UNKNOWN_DRIVECODE, datacrc >> 8, datacrc & 0xff);

  datacrc = 0xffff;
  previous_loader = detected_loader;
  detected_loader = FL_NONE;
}


/* ------------------------------------------------------------------------- */
/*  Parsing helpers                                                          */
/* ------------------------------------------------------------------------- */

/* Fill the end of the command buffer with 0x00  */
/* so C string functions can work on file names. */

static void clean_cmdbuffer(void) {
  memset(command_buffer+command_length, 0, sizeof(command_buffer)-command_length);
}


/* Parse parameters of block commands in the command buffer */
/* Returns number of parameters (up to 4) or <0 on error    */
static int8_t parse_blockparam(uint8_t values[]) {
  uint8_t paramcount = 0;
  uint8_t *str;

  str = ustrchr(command_buffer, ':');
  if (!str) {
    if (ustrlen(command_buffer) < 3)
      return -1;
    str = command_buffer + 2;
  }

  str++;

  while (*str && paramcount < 4) {
    /* Skip all spaces, cursor-rights and commas - CC7C */
    while (*str == ' ' || *str == CURSOR_RIGHT || *str == ',') str++;
    if (!*str)
      break;

    values[paramcount++] = parse_number(&str);
  }

  return paramcount;
}

static uint8_t parse_bool(void) {
  switch (command_buffer[2]) {
  case '+':
    return 1;

  case '-':
    return 0;

  default:
    set_error(ERROR_SYNTAX_UNKNOWN);
    return 255;
  }
}

/* ------------------------------------------------------------------------- */
/*  Command handlers                                                         */
/* ------------------------------------------------------------------------- */

/* ------------------- */
/*  CD/MD/RD commands  */
/* ------------------- */

/* --- MD --- */
static void parse_mkdir(void) {
  path_t  path;
  uint8_t *name;

  /* MD requires a colon */
  if (!ustrchr(command_buffer, ':')) {
    set_error(ERROR_SYNTAX_NONAME);
    return;
  }
  if (parse_path(command_buffer+2, &path, &name, 0))
    return;
  mkdir(&path,name);
}

/* --- CD --- */

/* internal version of CD that is used by CD and diskchange.c */
void do_chdir(uint8_t *parsestr) {
  path_t      path;
  uint8_t    *name;
  cbmdirent_t dent;

  if (parse_path(parsestr, &path, &name, 1))
    return;

  /* clear '*' file */
  previous_file_dirent.name[0] = 0;

  if (ustrlen(name) != 0) {
    /* Path component after the : */
    if (name[0] == '_') {
      /* Going up a level */
      ustrcpy(dent.name, name);
      if (chdir(&path,&dent))
        return;
    } else {
      /* A directory name - try to match it */
      if (first_match(&path, name, FLAG_HIDDEN, &dent))
        return;

      if (chdir(&path, &dent))
        return;
    }
  } else {
    /* reject if there is no / in the string */
    if (!ustrchr(parsestr, '/')) {
      set_error(ERROR_FILE_NOT_FOUND_39);
      return;
    }
  }

  update_current_dir(&path);
}

static void parse_chdir(void) {
  do_chdir(command_buffer + 2);

  if (globalflags & AUTOSWAP_ACTIVE)
    set_changelist(NULL, NULLSTRING);
}

/* --- CD/MD/RD subparser --- */
static void parse_dircommand(void) {
  clean_cmdbuffer();

  switch (command_buffer[0]) {
  case 'M':
    parse_mkdir();
    break;

  case 'C':
    parse_chdir();
    break;

  default:
    set_error(ERROR_SYNTAX_UNKNOWN);
    break;
  }
}


/* ------------ */
/*  B commands  */
/* ------------ */
static void parse_block(void) {
  uint8_t  *str;
  buffer_t *buf;
  uint8_t  params[4];
  int8_t   pcount;

  clean_cmdbuffer();

  str = ustrchr(command_buffer, '-');
  if (!str) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  memset(params,0,sizeof(params));
  pcount = parse_blockparam(params);
  if (pcount < 0)
    return;

  str++;
  switch (*str) {
  case 'R':
  case 'W':
    /* Block-Read  - CD56 */
    /* Block-Write - CD73 */
    buf = find_buffer(params[0]);
    if (!buf) {
      set_error(ERROR_NO_CHANNEL);
      return;
    }

    if (*str == 'R') {
      read_sector(buf, buf->pvt.buffer.part, params[2], params[3]);
      if (command_buffer[0] == 'B') {
        buf->position = 1;
        buf->lastused = buf->data[0];
      } else {
        buf->position = 0;
        buf->lastused = 255;
      }
    } else {
      if (command_buffer[0] == 'B')
        buf->data[0] = buf->position-1; // FIXME: Untested, verify!
      write_sector(buf, buf->pvt.buffer.part, params[2], params[3]);
    }
    break;

  case 'P':
    /* Buffer-Position - CDBD */
    buf = find_buffer(params[0]);
    if (!buf) {
      set_error(ERROR_NO_CHANNEL);
      return;
    }
    if (buf->pvt.buffer.size != 1) {
      /* Extended position for large buffers */
      uint8_t count = params[2];

      /* Walk the chain, wrap whenever necessary */
      buf->secondary = BUFFER_SEC_CHAIN - params[0];
      buf = buf->pvt.buffer.first;
      while (count--) {
        if (buf->pvt.buffer.next != NULL)
          buf = buf->pvt.buffer.next;
        else
          buf = buf->pvt.buffer.first;
      }
      buf->secondary = params[0];
      buf->mustflush = 0;
    }
    buf->position = params[1];
    break;

  default:
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }
}

/* ------------ */
/*  D commands  */
/* ------------ */
static void parse_direct(void) {
    set_error(ERROR_SYNTAX_UNABLE);
}


/* -------------------------- */
/*  G-P - Get Partition info  */
/* -------------------------- */
static void parse_getpartition(void) {
  uint8_t *ptr;
  uint8_t part;

  if (command_length < 3) /* FIXME: should this set an error? */
    return;

  if (command_buffer[1] != '-' || command_buffer[2] != 'P') {
    set_error(ERROR_SYNTAX_UNKNOWN);
    return;
  }

  if (command_length == 3 || command_buffer[3] == 0xff)
    part = current_part + 1;
  else
    part = command_buffer[3];

  buffers[ERRORBUFFER_IDX].position = 0;
  buffers[ERRORBUFFER_IDX].lastused = 30;

  memset(error_buffer,0,30);
  error_buffer[30] = 13;
  ptr = error_buffer;

  if (part > max_part) {
    /* Nonexisting partition - return empty answer */
    ptr[30] = 13;
    return;
  }

  if (part == 0) {
    /* System partition - return static info */
    memcpy_P(ptr, system_partition_info, sizeof(system_partition_info));
    return;
  }

  part -= 1;

  /* Create partition info */
  if (partition[part].fop == &d64ops) {
    /* Use type of mounted image as partition type */
    *ptr++ = partition[part].imagetype & D64_TYPE_MASK;
  } else {
    /* Use native for anything else */
    *ptr++ = 1;
  }
  *ptr++ = 0xe2; // 1.6MB disk - "reserved" for HD

  *ptr++ = part+1;

  /* Read partition label */
  memset(ptr, 0xa0, 16);
  if (disk_label(part, ops_scratch)) {
    return;
  }

  uint8_t *inptr  = ops_scratch;
  uint8_t *outptr = ptr;
  while (*inptr)
    *outptr++ = *inptr++;

  ptr += 16;
  *ptr++ = (partition[part].fatfs.fatbase >> 16) & 0xff;
  *ptr++ = (partition[part].fatfs.fatbase >>  8) & 0xff;
  *ptr++ = (partition[part].fatfs.fatbase      ) & 0xff;
  ptr += 5; // reserved bytes

  uint32_t size;

  if (partition[part].fop == &d64ops) {
    /* return the size of the disk image, rounded up */
    size = (partition[part].imagehandle.fsize + 511) / 512;

  } else {
    /* return the size of the FAT partition */
    size = (partition[part].fatfs.max_clust - 1) * partition[part].fatfs.csize;
  }

  if (size > 0xffffff)
    size = 0xffffff;

  *ptr++ = (size >> 16) & 0xff;
  *ptr++ = (size >>  8) & 0xff;
  *ptr   = size & 0xff;
}


/* ---------------- */
/*  I - Initialize  */
/* ---------------- */
static void parse_initialize(void) {
  if (disk_state != DISK_OK)
    set_error_ts(ERROR_READ_NOSYNC,18,0);
  else
    free_multiple_buffers(FMB_USER_CLEAN);
}


/* ------------ */
/*  M commands  */
/* ------------ */

/* --- M subparser --- */
static void parse_memory(void) {
  set_error(ERROR_SYNTAX_UNKNOWN);
}

/* --------- */
/*  N - New  */
/* --------- */
static void parse_new(void) {
  uint8_t *name, *id;
  uint8_t part;

  clean_cmdbuffer();

  name = command_buffer+1;
  part = parse_partition(&name);
  name = ustrchr(command_buffer, ':');
  if (name++ == NULL) {
    set_error(ERROR_SYNTAX_NONAME);
    return;
  }

  id = ustrchr(name, ',');
  if (id != NULL) {
    *id = 0;
    id++;
  }

  format(part, name, id);
}


/* -------------- */
/*  P - Position  */
/* -------------- */
static void parse_position(void) {
  buffer_t *buf;

  command_length = original_length;
  clean_cmdbuffer();

  if(command_length < 2 || (buf = find_buffer(command_buffer[1] & 0x0f)) == NULL) {
    set_error(ERROR_NO_CHANNEL);
    return;
  }

  if (buf->seek == NULL) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  if (buf->recordlen) {
    /* REL file */
    uint16_t record;
    uint8_t  pos;
    record = 1;
    pos = 1;

    if (command_length > 1)
      record = command_buffer[2];
    if (command_length > 2)
      record |= command_buffer[3] << 8;
    if (command_length > 3)
      pos = command_buffer[4];

    if (pos > buf->recordlen) {
      set_error(ERROR_RECORD_OVERFLOW);
      return;
    }

    if (record)
      record--;

    if (pos)
      pos--;

    buf->seek(buf, record * (uint32_t)buf->recordlen, pos);
  } else {
    /* Non-REL seek uses a straight little-endian offset */
    union {
      uint32_t l;
      uint8_t  c[4];
    } offset;

    // smaller than memcpy
    /* WARNING: Endian-dependant */
    offset.c[0] = command_buffer[2];
    offset.c[1] = command_buffer[3];
    offset.c[2] = command_buffer[4];
    offset.c[3] = command_buffer[5];

    buf->seek(buf, offset.l, 0);
  }
}


/* ------------ */
/*  R - Rename  */
/* ------------ */
static void parse_rename(void) {
  set_error(ERROR_OK);
}


/* ------------- */
/*  S - Scratch  */
/* ------------- */
static void parse_scratch(void) {
  set_error_ts(ERROR_SCRATCHED,0,0);
}


#ifdef HAVE_RTC
/* ------------------ */
/*  T - Time commands */
/* ------------------ */

/* --- T-R --- */
static void parse_timeread(void) {
  struct tm time;
  uint8_t *ptr = error_buffer;
  uint8_t hour;

  if (rtc_state != RTC_OK) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  read_rtc(&time);
  hour = time.tm_hour % 12;
  if (hour == 0) hour = 12;

  switch (command_buffer[3]) {
  case 'A': /* ASCII format */
    buffers[ERRORBUFFER_IDX].lastused = 25;
    memcpy_P(error_buffer+4, asciitime_skel, sizeof(asciitime_skel));
    memcpy_P(error_buffer, downames + 4*time.tm_wday, 4);
    appendnumber(error_buffer+5, time.tm_mon+1);
    appendnumber(error_buffer+8, time.tm_mday);
    appendnumber(error_buffer+11, time.tm_year % 100);
    appendnumber(error_buffer+14, hour);
    appendnumber(error_buffer+17, time.tm_min);
    appendnumber(error_buffer+20, time.tm_sec);
    if (time.tm_hour < 12)
      error_buffer[23] = 'A';
    else
      error_buffer[23] = 'P';
    break;

  case 'B': /* BCD format */
    buffers[ERRORBUFFER_IDX].lastused = 8;
    *ptr++ = time.tm_wday;
    *ptr++ = int2bcd(time.tm_year % 100);
    *ptr++ = int2bcd(time.tm_mon+1);
    *ptr++ = int2bcd(time.tm_mday);
    *ptr++ = int2bcd(hour);
    *ptr++ = int2bcd(time.tm_min);
    *ptr++ = int2bcd(time.tm_sec);
    *ptr++ = (time.tm_hour >= 12);
    *ptr   = 13;
    break;

  case 'D': /* Decimal format */
    buffers[ERRORBUFFER_IDX].lastused = 8;
    *ptr++ = time.tm_wday;
    *ptr++ = time.tm_year;
    *ptr++ = time.tm_mon+1;
    *ptr++ = time.tm_mday;
    *ptr++ = hour;
    *ptr++ = time.tm_min;
    *ptr++ = time.tm_sec;
    *ptr++ = (time.tm_hour >= 12);
    *ptr   = 13;
    break;

  case 'I': /* ISO 8601 format plus DOW */
    buffers[ERRORBUFFER_IDX].lastused = 23;
    ptr = appendnumber(ptr, 19 + (time.tm_year / 100));
    ptr = appendnumber(ptr, time.tm_year % 100);
    *ptr++ = '-';
    ptr = appendnumber(ptr, time.tm_mon + 1);
    *ptr++ = '-';
    ptr = appendnumber(ptr, time.tm_mday);
    *ptr++ = 'T';
    ptr = appendnumber(ptr, time.tm_hour);
    *ptr++ = ':';
    ptr = appendnumber(ptr, time.tm_min);
    *ptr++ = ':';
    ptr = appendnumber(ptr, time.tm_sec);
    *ptr   = ' ';
    memcpy_P(error_buffer + 20, downames + 4*time.tm_wday, 3);
    error_buffer[23] = 13;
    break;

  default: /* Unknown format */
    set_error(ERROR_SYNTAX_UNKNOWN);
    break;
  }
}

/* --- T-W --- */
static void parse_timewrite(void) {

  /* day of week calculation by M. Keith and T. Craver via */
  /* https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week */
  uint8_t day_of_week(uint16_t y, uint8_t m, uint8_t d) {
    y += 1900;
    return (d+=m<3?y--:y-2,23*m/9+d+4+y/4-y/100+y/400)%7;
  }

  struct tm time;
  uint8_t i, *ptr;

  switch (command_buffer[3]) {
  case 'A': /* ASCII format */
    if (command_length < 27) { // Allow dropping the AM/PM marker for 24h format
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }
    for (i=0;i<7;i++) {
      if (memcmp_P(command_buffer+4, downames + 4*i, 2) == 0) // only need to compare 2
        break;
    }
    if (i == 7) {
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }
    time.tm_wday = i;
    ptr = command_buffer + 9;
    time.tm_mon  = parse_number(&ptr);
    ptr++;
    time.tm_mday = parse_number(&ptr);
    ptr++;
    time.tm_year = parse_number(&ptr);
    ptr++;
    time.tm_hour = parse_number(&ptr);
    ptr++;
    time.tm_min  = parse_number(&ptr);
    ptr++;
    time.tm_sec  = parse_number(&ptr);
    if (command_buffer[28] == 'M') {
      /* Adjust for AM/PM only if AM/PM is actually supplied */
      if (time.tm_hour == 12)
        time.tm_hour = 0;
      if (command_buffer[27] == 'P')
        time.tm_hour += 12;
    }
    break;

  case 'B': /* BCD format */
    if (command_length < 12) {
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }
    time.tm_wday = command_buffer[4];
    time.tm_year = bcd2int(command_buffer[5]);
    time.tm_mon  = bcd2int(command_buffer[6]);
    time.tm_mday = bcd2int(command_buffer[7]);
    time.tm_hour = bcd2int(command_buffer[8]);
    /* Hour range is 1-12, change 12:xx to 0:xx for easier conversion */
    if (time.tm_hour == 12)
      time.tm_hour = 0;
    time.tm_min  = bcd2int(command_buffer[9]);
    time.tm_sec  = bcd2int(command_buffer[10]);
    if (command_buffer[11])
      time.tm_hour += 12;
    break;

  case 'D': /* Decimal format */
    if (command_length < 12) {
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }
    time.tm_wday = command_buffer[4];
    time.tm_year = command_buffer[5];
    time.tm_mon  = command_buffer[6];
    time.tm_mday = command_buffer[7];
    time.tm_hour = command_buffer[8];
    /* Hour range is 1-12, change 12:xx to 0:xx for easier conversion */
    if (time.tm_hour == 12)
      time.tm_hour = 0;
    time.tm_min  = command_buffer[9];
    time.tm_sec  = command_buffer[10];
    if (command_buffer[11])
      time.tm_hour += 12;
    break;

  case 'I': /* ISO 8601 format, calculated day of week */
    if (command_length < 23) {
      set_error(ERROR_SYNTAX_UNABLE);
      return;
    }

    ptr = command_buffer + 4;
    time.tm_year = parse_number(&ptr) - 1900;
    ptr++;
    time.tm_mon  = parse_number(&ptr);
    ptr++;
    time.tm_mday = parse_number(&ptr);
    ptr++;
    time.tm_hour = parse_number(&ptr);
    ptr++;
    time.tm_min  = parse_number(&ptr);
    ptr++;
    time.tm_sec  = parse_number(&ptr);
    time.tm_wday = day_of_week(time.tm_year, time.tm_mon, time.tm_mday);
    break;

  default: /* Unknown format */
    set_error(ERROR_SYNTAX_UNKNOWN);
    return;
  }

  /* final adjustment of month to 0-11 range */
  time.tm_mon--;

  /* Y2K fix for legacy apps */
  if (time.tm_year < 80)
    time.tm_year += 100;

  /* The CMD drives don't check for validity, we do - partially */
  if (time.tm_mday ==  0 || time.tm_mday >  31 ||
      time.tm_mon  >  11 ||
      time.tm_wday >   6 ||
      time.tm_hour >  23 ||
      time.tm_min  >  59 ||
      time.tm_sec  >  59) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  set_rtc(&time);
}

/* --- T subparser --- */
static void parse_time(void) {
  if (rtc_state == RTC_NOT_FOUND)
    set_error(ERROR_SYNTAX_UNKNOWN);
  else {
    if (command_buffer[2] == 'R') {
      parse_timeread();
    } else if (command_buffer[2] == 'W') {
      parse_timewrite();
    } else
      set_error(ERROR_SYNTAX_UNKNOWN);
  }
}
#endif /* HAVE_RTC */


/* ------------ */
/*  U commands  */
/* ------------ */
static void parse_user(void) {
  switch (command_buffer[1]) {
  case 'A':
  case '1':
    /* Tiny little hack: Rewrite as (B)-R and call that                */
    /* This will always work because there is either a : in the string */
    /* or the drive will start parsing at buf[3].                      */
    command_buffer[0] = '-';
    command_buffer[1] = 'R';
    parse_block();
    break;

  case 'B':
  case '2':
    /* Tiny little hack: see above case for rationale */
    command_buffer[0] = '-';
    command_buffer[1] = 'W';
    parse_block();
    break;

  case 'C': case 'D': case 'E': case 'F': case 'G': case 'H':
  case '3': case '4': case '5': case '6': case '7': case '8':
    /* start program in buffer */
    run_loader(0x500 + 3 * (command_buffer[1] - '3'));
    break;

  case 'I':
  case '9':
    if (command_length == 2) {
      /* Soft-reset - just return the dos version */
      set_error(ERROR_DOSVERSION);
      return;
    }
    switch (command_buffer[2]) {
    case '+':
      globalflags &= (uint8_t)~VC20MODE;
      break;

    case '-':
      globalflags |= VC20MODE;
      break;

    default:
      /* Soft-reset - just return the dos version */
      set_error(ERROR_DOSVERSION);
      break;
    }
    break;

  case 'J':
  case ':':
    /* Reset - technically hard-reset */
    /* Faked because Ultima 5 sends UJ. */
    free_multiple_buffers(FMB_USER);
    set_error(ERROR_DOSVERSION);
    break;

  case 202: /* Shift-J */
    /* The real hard reset command */
    system_reset();
    break;

  case '0':
    /* U0 - only device address changes for now */
    if ((command_buffer[2] & 0x1f) == 0x1e &&
        command_buffer[3] >= 4 &&
        command_buffer[3] <= 30) {
      device_address = command_buffer[3];
      display_address(device_address);
      break;
    }
    /* Fall through */

  default:
    set_error(ERROR_SYNTAX_UNKNOWN);
    break;
  }
}


/* ------------ */
/*  X commands  */
/* ------------ */
static void parse_xcommand(void) {
  uint8_t num;
  uint8_t *str;
  path_t path;

  clean_cmdbuffer();

  switch (command_buffer[1]) {
  case 'E':
    /* Change file extension mode */
    str = command_buffer+2;
    if (*str == '+') {
      globalflags |= EXTENSION_HIDING;
    } else if (*str == '-') {
      globalflags &= (uint8_t)~EXTENSION_HIDING;
    } else {
      num = parse_number(&str);
      if (num > 4) {
        set_error(ERROR_SYNTAX_UNKNOWN);
      } else {
        file_extension_mode = num;
        if (num >= 3)
          globalflags |= EXTENSION_HIDING;
      }
    }
    set_error_ts(ERROR_STATUS,device_address,0);
    break;

  case 'D':
    /* drive config */
#ifdef NEED_DISKMUX
    str = command_buffer+2;
    if(*str == '?') {
      set_error_ts(ERROR_STATUS,device_address,1);
      break;
    }
    num = parse_number(&str);
    if(num < 8) {
      while(*str == ' ')
        str++;
      if(*str == '=') {
        uint8_t val, i;
        str++;
        val = parse_number(&str);
        if(val <= 0x0f) {
          for(i = 0;(val != 0x0f) && i < 8; i++) {
            if(i != num && map_drive(num) == val) {
              /* Trying to set the same drive mapping in two places. */
              set_error(ERROR_SYNTAX_UNKNOWN);
              break;
            }
          }
          switch(val >> DRIVE_BITS) {
          case DISK_TYPE_NONE:
# ifdef HAVE_SD
          case DISK_TYPE_SD:
# endif
# ifdef HAVE_ATA
          case DISK_TYPE_ATA:
# endif
            if(map_drive(num) != val) {
              set_map_drive(num,val);
              /* sanity check.  If the user has truly turned off all drives, turn the
               * defaults back on
               */
              if(drive_config == 0xffffffff)
                set_drive_config(get_default_driveconfig());
              filesystem_init(0);
            }
            break;
          default:
            set_error(ERROR_SYNTAX_UNKNOWN);
            break;
          }
          break;
        }
      }
    } else
      set_error(ERROR_SYNTAX_UNKNOWN);
#else
    // return error for units without MUX support
    set_error(ERROR_SYNTAX_UNKNOWN);
#endif
    break;

  case 'I':
    /* image-as-directory mode */
    str = command_buffer + 2;
    num = parse_number(&str);
    if (num <= 2) {
      image_as_dir = num;
    } else {
      set_error(ERROR_SYNTAX_UNKNOWN);
    }
    break;

  case 'W':
    /* Write configuration */
    write_configuration();
    set_error_ts(ERROR_STATUS,device_address,0);
    break;

  case 'R':
    /* Set Rom-file */
    if (command_buffer[2] == ':') {
      if (command_length > ROM_NAME_LENGTH+3) {
        set_error(ERROR_SYNTAX_TOOLONG);
      } else {
        ustrcpy(rom_filename, command_buffer+3);
      }
    } else {
      /* Clear rom name */
      rom_filename[0] = 0;
    }
    break;

  case 'S':
    /* Swaplist */
    if (parse_path(command_buffer+2, &path, &str, 0))
      return;

    set_changelist(&path, str);
    break;

  case '*':
    /* Post-* matching */
    num = parse_bool();
    if (num != 255) {
      if (num)
        globalflags |= POSTMATCH;
      else
        globalflags &= (uint8_t)~POSTMATCH;

      set_error_ts(ERROR_STATUS,device_address,0);
    }
    break;

#ifdef CONFIG_STACK_TRACKING
  case '?':
    /* Output the largest stack size seen */
    //FIXME: AVR-only code
    set_error_ts(ERROR_LONGVERSION,(RAMEND-minstack)>>8,(RAMEND-minstack)&0xff);
    break;
#else
  case '?':
    /* Output the long version string */
    set_error(ERROR_LONGVERSION);
    break;
#endif

#ifdef CONFIG_PARALLEL_DOLPHIN
  case 'Q': // fast load
    load_dolphin();
    break;

  case 'Z': // fast save
    save_dolphin();
    break;
#endif

  default:
    if (command_length != 1)
      /* unknown command */
      set_error(ERROR_SYNTAX_UNKNOWN);
    else
      /* plain X by itself, show the extended status */
      set_error_ts(ERROR_STATUS, device_address, 0);

    break;
  }
}


/* ------------------------------------------------------------------------- */
/*  Main command parser function                                             */
/* ------------------------------------------------------------------------- */

void parse_doscommand(void) {
  /* Set default message: Everything ok */
  set_error(ERROR_OK);

  /* Abort if the command is too long */
  if (command_length == CONFIG_COMMAND_BUFFER_SIZE) {
    set_error(ERROR_SYNTAX_TOOLONG);
    return;
  }

#ifdef CONFIG_COMMAND_CHANNEL_DUMP
  /* Debugging aid: Dump the whole command via serial */
  if (detected_loader == FL_NONE) {
    /* Dump only if no loader was detected because it may ruin the timing */
    uart_trace(command_buffer,0,command_length);
  }
#endif

  /* Remove one CR at end of command */
  original_length = command_length;
  if (command_length > 0 && command_buffer[command_length-1] == 0x0d)
    command_length--;

  /* Abort if there is no command */
  if (command_length == 0) {
    set_error(ERROR_SYNTAX_UNABLE);
    return;
  }

  /* Send command to display */
  display_doscommand(command_length, command_buffer);

  /* MD/CD/RD clash with other commands, so they're checked first */
  if (command_buffer[0] != 'X' && command_buffer[1] == 'D') {
    parse_dircommand();
    return;
  }

  switch (command_buffer[0]) {
  case 'B':
    /* Block-Something */
    parse_block();
    break;

  case 'D':
    /* Direct sector access (was duplicate in CBM drives) */
    parse_direct();
    break;

  case 'G':
    /* Get-Partition */
    parse_getpartition();
    break;

  case 'I':
    /* Initialize */
    parse_initialize();
    break;

  case 'M':
    /* Memory-something */
    parse_memory();
    break;

  case 'N':
    /* New */
    parse_new();
    break;

  case 'P':
    /* Position */
    parse_position();
    break;

  case 'R':
    /* Rename */
    parse_rename();
    break;

  case 'S':
    if(command_length == 3 && command_buffer[1] == '-') {
      /* Swap drive number */
      set_error(ERROR_SYNTAX_UNABLE);
      break;
    }
    /* Scratch */
    parse_scratch();
    break;

#ifdef HAVE_RTC
  case 'T':
    parse_time();
    break;
#endif

  case 'U':
    parse_user();
    break;

  case 'X':
    parse_xcommand();
    break;

  default:
    set_error(ERROR_SYNTAX_UNKNOWN);
    break;
  }
}
