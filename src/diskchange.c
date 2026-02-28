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


   diskchange.c: Disk image changer

*/

#include <stdbool.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "display.h"
#include "doscmd.h"
#include "errormsg.h"
#include "fatops.h"
#include "flags.h"
#include "ff.h"
#include "led.h"
#include "parser.h"
#include "progmem.h"
#include "timer.h"
#include "utils.h"
#include "ustring.h"
#include "diskchange.h"

#define BLINK_BACKWARD 1
#define BLINK_FORWARD  2
#define BLINK_HOME     3

static void confirm_blink(uint8_t type) {
  uint8_t i;

  for (i=0;i<2;i++) {
    tick_t targettime;

#ifdef SINGLE_LED
    set_dirty_led(1);
#else
    if (!i || type & 1)
      set_dirty_led(1);
    if (!i || type & 2)
      set_busy_led(1);
#endif
    targettime = ticks + MS_TO_TICKS(100);
    while (time_before(ticks,targettime)) ;

    set_dirty_led(0);
    set_busy_led(0);
    targettime = ticks + MS_TO_TICKS(100);
    while (time_before(ticks,targettime)) ;
  }
}

void set_changelist(path_t *path, uint8_t *filename) {
  (void)filename;
    (void)path;
}

void change_disk(void) {
  confirm_blink(BLINK_HOME);
}

void change_init(void) {
  
}
