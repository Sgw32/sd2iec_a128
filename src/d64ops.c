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


   d64ops.c: D64 operations

*/

#include <stdint.h>
#include <string.h>
#include "config.h"
#include "buffers.h"
#include "dirent.h"
#include "errormsg.h"
#include "fatops.h"
#include "ff.h"
#include "parser.h"
#include "progmem.h"
// #include "rtc.h"
#include "wrapops.h"
#include "d64ops.h"

#define D41_ERROR_OFFSET 174848

#define D41_BAM_TRACK           18
#define D41_BAM_SECTOR          0
#define D41_BAM_BYTES_PER_TRACK 4

/* used for error info only */
#define MAX_SECTORS_PER_TRACK 40

typedef enum { BAM_BITFIELD, BAM_FREECOUNT } bamdata_t;

struct {
  uint8_t part;
  uint8_t track;
  uint8_t errors[MAX_SECTORS_PER_TRACK];
} errorcache;

static buffer_t *bam_buffer;  // recently-used buffer
static buffer_t *bam_buffer2; // secondary buffer
static uint8_t   bam_refcount;

/* ------------------------------------------------------------------------- */
/*  Forward declarations                                                     */
/* ------------------------------------------------------------------------- */

static uint8_t d64_opendir(dh_t *dh, path_t *path);

static void format_d41_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf);

/* ------------------------------------------------------------------------- */
/*  Utility functions                                                        */
/* ------------------------------------------------------------------------- */

static const PROGMEM struct param_s d41param = {
  18, 1, 35, 0x90, 0xa2, 10, 3, format_d41_image
};

/**
 * get_param - get a format-dependent parameter
 * @part : partition number
 * @param: parameter to retrieve
 *
 * Returns a parameter that is format-dependent for the image type mounted
 * on the specified partition.
 */
static uint8_t get_param(uint8_t part, param_t param) {
  return *(((uint8_t *)&partition[part].d64data)+param);
}

/**
 * sector_lba - Transform a track/sector pair into a LBA sector number
 * @part  : partition number
 * @track : Track number
 * @sector: Sector number
 *
 * Calculates an LBA-style sector number for a given track/sector pair.
 */
/* This version used the least code of all tested variants. */
static uint16_t sector_lba(uint8_t part, uint8_t track, const uint8_t sector) {
  uint16_t offset = 0;

  track--; /* Track numbers are 1-based */

  (void)part;
  if (track >= 35) {
    offset = 683;
    track -= 35;
  }
  if (track < 17)
    return track*21 + sector + offset;
  if (track < 24)
    return 17*21 + (track-17)*19 + sector + offset;
  if (track < 30)
    return 17*21 + 7*19 + (track-24)*18 + sector + offset;
  return 17*21 + 7*19 + 6*18 + (track-30)*17 + sector + offset;
}

/**
 * sector_offset - Transform a track/sector pair into a D64 offset
 * @part  : partition number
 * @track : Track number
 * @sector: Sector number
 *
 * Calculates an offset into a D64 file from a track and sector number.
 */
static uint32_t sector_offset(uint8_t part, uint8_t track, const uint8_t sector) {
  return 256L * sector_lba(part,track,sector);
}

/**
 * sectors_per_track - number of sectors on given track
 * @part : partition number
 * @track: Track number
 *
 * This function returns the number of sectors on the given track
 * of a 1541/71/81 disk. Invalid track numbers will return invalid results.
 */
static uint16_t sectors_per_track(uint8_t part, uint8_t track) {
  (void)part;
  if (track > 35)
    track -= 35;
  if (track < 18)
    return 21;
  if (track < 25)
    return 19;
  if (track < 31)
    return 18;
  return 17;
}

/**
 * checked_read - read a specified sector after range-checking
 * @part  : partition number
 * @track : track number to be read
 * @sector: sector number to be read
 * @buf   : pointer to where the data should be read to
 * @len   : number of bytes to be read
 * @error : error number to be flagged if the range check fails
 *
 * This function checks if the track and sector are within the
 * limits for the image format and calls image_read to read
 * the data if they are. Returns the result of image_read or
 * 2 if the range check failed.
 */
static uint8_t checked_read(uint8_t part, uint8_t track, uint8_t sector, uint8_t *buf, uint16_t len, uint8_t error) {
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= sectors_per_track(part, track)) {
    set_error_ts(error,track,sector);
    return 2;
  }

  if (partition[part].imagetype & D64_HAS_ERRORINFO) {
    /* Check if the sector is marked as bad */
    if (errorcache.part != part || errorcache.track != track) {
      /* Read the error info for this track */
      memset(errorcache.errors, 1, sizeof(errorcache.errors));

      if (image_read(part, D41_ERROR_OFFSET + sector_lba(part,track,0),
                     errorcache.errors, sectors_per_track(part, track)) >= 2)
        return 2;
      errorcache.part  = part;
      errorcache.track = track;
    }

    /* Calculate error message from the code */
    if (errorcache.errors[sector] >= 2 && errorcache.errors[sector] <= 11) {
      /* Most codes can be mapped directly */
      set_error_ts(errorcache.errors[sector]-2+20,track,sector);
      return 2;
    }
    if (errorcache.errors[sector] == 15) {
      /* Drive not ready */
      set_error(74);
      return 2;
    }
    /* 1 is OK, unknown values are accepted too */
  }

  return image_read(part, sector_offset(part,track,sector), buf, len);
}

/**
 * update_timestamp - update timestamp of a directory entry
 * @buffer: pointer to the directory entry
 *
 * This function updates the time stamp of a directory entry
 * at the address pointed to by @buffer with the current
 * date/time if available or the default timestamp if not.
 */
static void update_timestamp(uint8_t *buffer) {
#ifdef HAVE_RTC
  struct tm time;

  read_rtc(&time);
  buffer[DIR_OFS_YEAR]   = time.tm_year % 100;
  buffer[DIR_OFS_MONTH]  = time.tm_mon + 1;
  buffer[DIR_OFS_DAY]    = time.tm_mday;
  buffer[DIR_OFS_HOUR]   = time.tm_hour;
  buffer[DIR_OFS_MINUTE] = time.tm_min;
#else
  buffer[DIR_OFS_YEAR]   = 82;
  buffer[DIR_OFS_MONTH]  = 8;
  buffer[DIR_OFS_DAY]    = 31;
  buffer[DIR_OFS_HOUR]   = 0;
  buffer[DIR_OFS_MINUTE] = 0;
#endif
}

/**
 * write_entry - write a single directory entry
 * @part : partition number
 * @dh   : pointer to d64dh pointing to the entry
 * @buf  : pointer to the buffer where the data should be read from
 * @flush: if true, data is flushed to disk immediately
 *
 * This function writes a single directory entry specified by @dh
 * from the buffer @buf.
 * Assumes that it is never called with an invalid track/sector.
 * Returns the same as image_write
 */
static uint8_t write_entry(uint8_t part, struct d64dh *dh, uint8_t *buf, uint8_t flush) {
  return image_write(part, sector_offset(part, dh->track, dh->sector) +
                           dh->entry * 32, buf, 32, flush);
}

/**
 * read_entry - read a single directory entry
 * @part: partition number
 * @dh  : pointer to d64dh pointing to the entry
 * @buf : pointer to the buffer where the data should be stored
 *
 * This function reads a single directory entry specified by dh
 * into the buffer buf which needs to be at least 32 bytes big.
 * Assumes that it is never called with an invalid track/sector.
 * Returns the same as image_read (0 success, 1 partial read, 2 failed)
 */
static uint8_t read_entry(uint8_t part, struct d64dh *dh, uint8_t *buf) {
  return image_read(part, sector_offset(part, dh->track, dh->sector) +
                          dh->entry * 32, buf, 32);
}

/**
 * strnsubst - substitute one character with another in a buffer
 * @buffer : pointer to the buffer
 * @len    : length of buffer
 * @oldchar: character to be replaced
 * @newchar: character to be used as a replacement
 *
 * This functions changes all occurences of oldchar in the first len
 * byte of buffer with newchar. Although it is named str*, it doesn't
 * care about zero bytes in any way.
 */
static void strnsubst(uint8_t *buffer, uint8_t len, uint8_t oldchar, uint8_t newchar) {
  uint8_t i=len-1;

  do {
    if (buffer[i] == oldchar)
      buffer[i] = newchar;
  } while (i--);
}

/* fill a sector with 0 0xff 0* to generate a new empty directory sector */
/* needs a 256 byte work area in *data */
static uint8_t clear_dir_sector(uint8_t part, uint8_t t, uint8_t s, uint8_t *data) {
  memset(data, 0, 256);

  data[1] = 0xff;
  return image_write(part, sector_offset(part, t, s), data, 256, 0);
}


/* ------------------------------------------------------------------------- */
/*  BAM buffer handling                                                      */
/* ------------------------------------------------------------------------- */

/**
 * bam_buffer_flush - write BAM buffer to disk
 * @buf: pointer to the BAM buffer
 *
 * This function writes the contents of the BAM buffer to the disk image.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t bam_buffer_flush(buffer_t *buf) {
  uint8_t res;

  if (buf->mustflush && buf->pvt.bam.part < max_part) {
    res = image_write(buf->pvt.bam.part,
                      sector_offset(buf->pvt.bam.part,
                                    buf->pvt.bam.track,
                                    buf->pvt.bam.sector),
                      buf->data, 256, 1);
    buf->mustflush = 0;

    return res;
  } else
    return 0;
}

/**
 * d64_bam_commit - write BAM buffers to disk
 *
 * This function is the exported interface to force the BAM buffer
 * contents to disk. Returns 0 if successful, != 0 otherwise.
 */
uint8_t d64_bam_commit(void) {
  uint8_t res = 0;

  if (bam_buffer)
    res |= bam_buffer->cleanup(bam_buffer);

  if (bam_buffer2)
    res |= bam_buffer2->cleanup(bam_buffer2);

  return 0;
}

/**
 * bam_buffer_alloc - allocates a buffer for the BAM
 * @buf: pointer to the BAM buffer pointer
 *
 * This function tries to allocate a buffer for holding the BAM,
 * using the pointer pointed to by @buf. Returns 0 if successful,
 * != 0 otherwise.
 */
static uint8_t bam_buffer_alloc(buffer_t **buf) {
  *buf = alloc_system_buffer();
  if (!*buf)
    return 1;

  (*buf)->secondary    = BUFFER_SYS_BAM;
  (*buf)->pvt.bam.part = 255;
  (*buf)->cleanup      = bam_buffer_flush;
  stick_buffer(*buf);

  return 0;
}

/**
 * bam_buffer_swap - swap the two BAM buffers
 *
 * Thus function swaps the pointers to the two BAM buffers
 */
static void bam_buffer_swap(void) {
  buffer_t *tmp = bam_buffer;
  bam_buffer    = bam_buffer2;
  bam_buffer2   = tmp;
}

/**
 * bam_buffer_match - checks if a BAM buffer contains a specific sector
 * @buf   : BAM buffer to check
 * @part  : partition
 * @track : track
 * @sector: sector
 *
 * This function checks if the BAM buffer @buf currently holds the data
 * for the given @part, @track and @sector. Returns 1 if it does,
 * 0 otherwise.
 */
static uint8_t bam_buffer_match(buffer_t *buf, uint8_t part,
                                uint8_t track, uint8_t sector) {
  if (buf->pvt.bam.part   == part  &&
      buf->pvt.bam.track  == track &&
      buf->pvt.bam.sector == sector)
    return 1;
  else
    return 0;
}

/**
 * move_bam_window - read correct BAM sector into window.
 * @part  : partition
 * @track : track number
 * @type  : type of pointer requested
 * @ptr   : pointer to track information in BAM sector
 *
 * This function reads the correct BAM sector into memory for the requested
 * track, flushing an existing BAM sector to disk if needed.  It also
 * calculates the correct pointer into the BAM sector for the appropriate
 * track.  Since the BAM contains both sector counts and sector allocation
 * bitmaps, type is used to signal which reference is desired.
 * This function may swap the BAM buffer pointers, after it returns
 * bam_buffer is always the buffer with the requested sector.
 * Returns 0 if successful, != 0 otherwise.
 */
static uint8_t move_bam_window(uint8_t part, uint8_t track, bamdata_t type, uint8_t **ptr) {
  uint8_t res;
  uint8_t t = D41_BAM_TRACK;
  uint8_t s = D41_BAM_SECTOR;
  uint8_t pos = D41_BAM_BYTES_PER_TRACK * track + (type == BAM_BITFIELD ? 1:0);

  if (!bam_buffer_match(bam_buffer, part, t, s)) {
    /* check if the second BAM buffer exists */
    if (bam_buffer2) {
      /* unconditionally swap to the other BAM buffer        */
      /* either it has the data or it was less-recently used */

      /* Note for future optimization:
       * If buf1 is invalid and buf2 valid but not matching this will swap and
       * use buf2 for the data instead of buf1 - using buf1
       * while keeping buf2 without swapping would be better.
       */
      bam_buffer_swap();

      /* check again, now in the other buffer */
      if (bam_buffer_match(bam_buffer, part, t, s))
        goto found;
    } else {
      /* allocate and swap to the second BAM buffer if the first one is valid */
      if (bam_buffer->pvt.bam.part != 255) {
        if (bam_buffer_alloc(&bam_buffer2)) {
          /* allocation failed, reset error and continue with just one buffer */
          set_error(ERROR_OK);
        } else {
          /* switch to the new buffer */
          bam_buffer_swap();
        }
      }
    }

    /* Need to read the BAM sector - flush only the target buffer */
    if (bam_buffer->cleanup(bam_buffer))
      return 1;

    res = image_read(part, sector_offset(part, t, s), bam_buffer->data, 256);
    if(res)
      return res;

    bam_buffer->pvt.bam.part   = part;
    bam_buffer->pvt.bam.track  = t;
    bam_buffer->pvt.bam.sector = s;
  }

 found:
  *ptr = bam_buffer->data + pos;
  return 0;
}

/**
 * is_free - checks if the given sector is marked as free
 * @part  : partition
 * @track : track number
 * @sector: sector number
 *
 * This function checks if the given sector is marked as free in
 * the BAM of drive "part". Returns 0 if allocated, >0 if free, <0 on error.
 */
static int8_t is_free(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t res;
  uint8_t *ptr = NULL;

  res = move_bam_window(part,track,BAM_BITFIELD,&ptr);
  if(res)
    return -1;

  return (ptr[sector>>3] & (1<<(sector&7))) != 0;
}

/**
 * sectors_free - returns the number of free sectors on a given track
 * @part  : partition
 * @track : track number
 *
 * This function returns the number of free sectors on the given track
 * of partition part.
 */
static uint16_t sectors_free(uint8_t part, uint8_t track) {
  uint8_t *trackmap = NULL;

  if (track < 1 || track > get_param(part, LAST_TRACK))
    return 0;

  if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
    return 0;
  return *trackmap;
}

/**
 * allocate_sector - mark a sector as used
 * @part  : partitoin
 * @track : track number
 * @sector: sector number
 *
 * This function marks the given sector as used in the BAM of drive part.
 * If the sector was already marked as used nothing is changed.
 * Returns 0 if successful, 1 on error.
 */
static uint8_t allocate_sector(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t *trackmap;
  int8_t res = is_free(part,track,sector);

  if (res < 0)
    return 1;

  if (res != 0) {
    /* do the bitfield first, since is_free already set it up for us. */
    if(move_bam_window(part,track,BAM_BITFIELD,&trackmap))
      return 1;

    bam_buffer->mustflush = 1;

    trackmap[sector>>3] &= (uint8_t)~(1<<(sector&7));

    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 1;

    if (trackmap[0] > 0) {
      trackmap[0]--;
      bam_buffer->mustflush = 1;
    }
  }
  return 0;
}

/**
 * free_sector - mark a sector as free
 * @part  : partition
 * @track : track number
 * @sector: sector number
 *
 * This function marks the given sector as free in the BAM of partition
 * part. If the sector was already marked as free nothing is changed.
 * Returns 0 if successful, 1 on error.
 */
static uint8_t free_sector(uint8_t part, uint8_t track, uint8_t sector) {
  uint8_t *trackmap;
  int8_t res = is_free(part,track,sector);

  if (res < 0)
    return 1;

  if (res == 0) {
    /* do the bitfield first, since is_free already set it up for us. */
    if(move_bam_window(part,track,BAM_BITFIELD,&trackmap))
      return 1;

    bam_buffer->mustflush = 1;

    trackmap[sector>>3] |= 1<<(sector&7);

    if(move_bam_window(part,track,BAM_FREECOUNT,&trackmap))
      return 1;

    if(trackmap[0] < sectors_per_track(part, track)) {
      trackmap[0]++;
      bam_buffer->mustflush = 1;
    }
  }
  return 0;
}

/**
 * get_first_sector - calculate the first sector for a new file
 * @part  : partition
 * @track : pointer to a variable holding the track
 * @sector: pointer to a variable holding the sector
 *
 * This function calculates the first sector to be allocated for a new
 * file. The algorithm is based on the description found at
 * http://ist.uwaterloo.ca/~schepers/formats/DISK.TXT
 * Returns 0 if successful or 1 if any error occured.
 *
 */
static uint8_t get_first_sector(uint8_t part, uint8_t *track, uint8_t *sector) {
  int8_t distance = 1;

  /* Look for a track with free sectors close to the directory */
  while (distance < get_param(part, LAST_TRACK)) {
    if (sectors_free(part, get_param(part, DIR_TRACK)-distance))
      break;

    /* Invert sign */
    distance = -distance;

    /* Increase distance every second try */
    if (distance > 0)
      distance++;
  }

  if (distance == get_param(part, LAST_TRACK)) {
    if (current_error == ERROR_OK)
      set_error(ERROR_DISK_FULL);
    return 1;
  }

  *track = get_param(part, DIR_TRACK)-distance;

  /* Search for the first free sector on this track */
  for (*sector = 0;*sector < sectors_per_track(part, *track); *sector += 1)
    if (is_free(part, *track, *sector) > 0)
      return 0;

  /* If we're here the BAM is invalid or couldn't be read */
  if (current_error == ERROR_OK)
    set_error(ERROR_DISK_FULL);
  return 1;
}

/**
 * get_next_sector - calculate the next sector for a file
 * @part  : partition
 * @track : pointer to a variable holding the track
 * @sector: pointer to a variable holding the sector
 *
 * This function calculates the next sector to be allocated for a file
 * based on the current sector in the variables pointed to by track/sector.
 * The algorithm is based on the description found at
 * http://ist.uwaterloo.ca/~schepers/formats/DISK.TXT
 * Returns 0 if successful or 1 if any error occured.
 */
static uint8_t get_next_sector(uint8_t part, uint8_t *track, uint8_t *sector) {
  uint8_t interleave,tries;

  if (*track == get_param(part, DIR_TRACK)) {
    if (sectors_free(part, get_param(part, DIR_TRACK)) == 0) {
      if (current_error == ERROR_OK)
        set_error(ERROR_DISK_FULL);
      return 1;
    }
    interleave = get_param(part, DIR_INTERLEAVE);
  } else
    interleave = get_param(part, FILE_INTERLEAVE);

  /* Look for a track with free sectors */
  tries = 0;
  while (tries < 3 && !sectors_free(part, *track)) {
    /* No more space on current track, try another */
    if (*track < get_param(part, DIR_TRACK))
      *track -= 1;
    else {
      *track += 1;
    }

    if (*track < 1) {
      *track = get_param(part, DIR_TRACK) + 1;
      *sector = 0;
      tries++;
    }
    if (*track > get_param(part, LAST_TRACK)) {
      *track = get_param(part, DIR_TRACK) - 1;
      *sector = 0;
      tries++;
    }
  }

  if (tries == 3) {
    if (current_error == ERROR_OK)
      set_error(ERROR_DISK_FULL);
    return 1;
  }

  /* Look for a sector at interleave distance */
  *sector += interleave;
  if (*sector >= sectors_per_track(part, *track)) {
    *sector -= sectors_per_track(part, *track);
    if (*sector != 0)
      *sector -= 1;
  }

  /* Increase distance until an empty sector is found */
  tries = 99;
  while (is_free(part,*track,*sector) <= 0 && tries--) {
    *sector += 1;
    if (*sector >= sectors_per_track(part, *track))
      *sector = 0;
  }

  if (tries)
    return 0;

  if (current_error == ERROR_OK)
    set_error(ERROR_DISK_FULL);
  return 1;
}

/**
 * nextdirentry - read the next dir entry to ops_scratch
 * @dh: directory handle
 *
 * This function reads the next directory entry from the disk
 * into ops_scratch. Returns 1 if an error occured, -1 if there
 * are no more directory entries and 0 if successful. This
 * function will return every entry, even deleted ones.
 */
static int8_t nextdirentry(dh_t *dh) {
  /* End of directory entries in this sector? */
  if (dh->dir.d64.entry == 8) {
    /* Read link pointer */
    if (checked_read(dh->part, dh->dir.d64.track, dh->dir.d64.sector, ops_scratch, 2, ERROR_ILLEGAL_TS_LINK))
      return 1;

    /* Final directory sector? */
    if (ops_scratch[0] == 0)
      return -1;

    dh->dir.d64.track  = ops_scratch[0];
    dh->dir.d64.sector = ops_scratch[1];
    dh->dir.d64.entry  = 0;
  }

  if (dh->dir.d64.track < 1 || dh->dir.d64.track > get_param(dh->part, LAST_TRACK) ||
      dh->dir.d64.sector >= sectors_per_track(dh->part, dh->dir.d64.track)) {
    set_error_ts(ERROR_ILLEGAL_TS_LINK,dh->dir.d64.track,dh->dir.d64.sector);
    return 1;
  }

  if (read_entry(dh->part, &dh->dir.d64, ops_scratch))
    return 1;

  dh->dir.d64.entry++;

  return 0;
}

/**
 * find_empty_entry - find an empty directory entry
 * @path: path of the directory
 * @dh: directory handle
 *
 * This funtion finds an empty entry in the directory, creating
 * a new directory sector if required.
 * Returns 0 if successful or 1 if not.
 */
static uint8_t find_empty_entry(path_t *path, dh_t *dh) {
  int8_t res;

  d64_opendir(dh, path);
  do {
    res = nextdirentry(dh);
    if (res > 0)
      return 1;
  } while (res == 0 && ops_scratch[DIR_OFS_FILE_TYPE] != 0);

  /* Allocate a new directory sector if no empty entries were found */
  if (res < 0) {
    uint8_t t,s;

    t = dh->dir.d64.track;
    s = dh->dir.d64.sector;

    if (get_next_sector(path->part, &dh->dir.d64.track, &dh->dir.d64.sector))
      return 1;

    /* Link the old sector to the new */
    ops_scratch[0] = dh->dir.d64.track;
    ops_scratch[1] = dh->dir.d64.sector;
    if (image_write(path->part, sector_offset(path->part,t,s), ops_scratch, 2, 0))
      return 1;

    if (allocate_sector(path->part, dh->dir.d64.track, dh->dir.d64.sector))
      return 1;

    /* Clear the new directory sector */
    memset(ops_scratch, 0, 32);
    ops_scratch[1] = 0xff;
    for (uint8_t i=0;i<256/32;i++) {
      dh->dir.d64.entry = i;
      if (write_entry(path->part, &dh->dir.d64, ops_scratch, 0))
        return 1;

      ops_scratch[1] = 0;
    }

    /* Mark full sector as used */
    ops_scratch[1] = 0xff;
    dh->dir.d64.entry = 0;

  } else {
    /* nextdirentry has already incremented this variable, undo it */
    dh->dir.d64.entry--;
  }

  return 0;
}

/**
 * d64_read - refill-callback used for reading
 * @buf: target buffer
 *
 * This is the callback used as refill for files opened for reading.
 */
static uint8_t d64_read(buffer_t *buf) {
  /* Store the current sector, used for append */
  buf->pvt.d64.track  = buf->data[0];
  buf->pvt.d64.sector = buf->data[1];

  if (checked_read(buf->pvt.d64.part, buf->data[0], buf->data[1], buf->data, 256, ERROR_ILLEGAL_TS_LINK)) {
    free_buffer(buf);
    return 1;
  }

  buf->position = 2;

  if (buf->data[0] == 0) {
    /* Final sector of the file */
    buf->lastused = buf->data[1];
    buf->sendeoi  = 1;
  } else {
    buf->lastused = 255;
    buf->sendeoi  = 0;
  }

  return 0;
}

/**
 * d64_seek - seek-callback
 * @buf     : target buffer
 * @position: offset to seek to
 * @index   : offset within the record to seek to
 *
 * This is the function used as the seek callback. Since seeking
 * isn't supported for D64 files it just sets an error message
 * and returns 1.
 */
static uint8_t d64_seek(buffer_t *buf, uint32_t position, uint8_t index) {
  set_error(ERROR_SYNTAX_UNABLE);
  return 1;
}

/**
 * d64_write - refill-callback used for writing
 * @buf: target buffer
 *
 * This is the callback used as refill for files opened for writing.
 */
static uint8_t d64_write(buffer_t *buf) {
  uint8_t t, s, savederror;

  savederror = 0;
  t = buf->pvt.d64.track;
  s = buf->pvt.d64.sector;

  buf->pvt.d64.blocks++;

  /* Mark as last sector in case something below fails */
  buf->data[0] = 0;
  buf->data[1] = buf->lastused;

  /* Find another free sector */
  if (get_next_sector(buf->pvt.d64.part, &t, &s)) {
    t = 0;
    savederror = current_error;
    goto storedata;
  }

  buf->data[0] = t;
  buf->data[1] = s;

  /* Allocate it */
  if (allocate_sector(buf->pvt.d64.part, t, s)) {
    free_buffer(buf);
    return 1;
  }

 storedata:
  /* Store data in the already-reserved sector */
  if (image_write(buf->pvt.d64.part,
                  sector_offset(buf->pvt.d64.part,
                                buf->pvt.d64.track,
                                buf->pvt.d64.sector),
                  buf->data, 256, 1)) {
    free_buffer(buf);
    return 1;
  }

  buf->pvt.d64.track  = t;
  buf->pvt.d64.sector = s;
  buf->position  = 2;
  buf->lastused  = 1;
  buf->mustflush = 0;
  mark_buffer_clean(buf);

  if (savederror) {
    set_error(savederror);
    free_buffer(buf);
    return 1;
  } else
    return 0;
}

static uint8_t d64_write_cleanup(buffer_t *buf) {
  uint8_t t,s;

  buf->data[0] = 0;
  buf->data[1] = buf->lastused;

  t = buf->pvt.d64.track;
  s = buf->pvt.d64.sector;
  buf->pvt.d64.blocks++;

  /* Track=0 means that there was an error somewhere earlier */
  if (t == 0)
    return 1;

  /* Store data */
  if (image_write(buf->pvt.d64.part, sector_offset(buf->pvt.d64.part,t,s), buf->data, 256, 1))
    return 1;

  /* Update directory entry */
  if (read_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch))
    return 1;

  ops_scratch[DIR_OFS_FILE_TYPE] |= FLAG_SPLAT;
  ops_scratch[DIR_OFS_SIZE_LOW]   = buf->pvt.d64.blocks & 0xff;
  ops_scratch[DIR_OFS_SIZE_HI]    = buf->pvt.d64.blocks >> 8;
  update_timestamp(ops_scratch);

  if (write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1))
    return 1;

  buf->cleanup = callback_dummy;
  free_buffer(buf);

  return 0;
}


/* ------------------------------------------------------------------------- */
/*  fileops-API                                                              */
/* ------------------------------------------------------------------------- */

uint8_t d64_mount(path_t *path, uint8_t *name) {
  uint8_t imagetype;
  uint8_t part = path->part;
  uint32_t fsize = partition[part].imagehandle.fsize;

  switch (fsize) {
  case 174848:
    imagetype = D64_TYPE_D41;
    memcpy_P(&partition[part].d64data, &d41param, sizeof(struct param_s));
    break;

  case 175531:
    imagetype = D64_TYPE_D41 | D64_HAS_ERRORINFO;
    memcpy_P(&partition[part].d64data, &d41param, sizeof(struct param_s));
    break;

  default:
    (void)name;
    set_error(ERROR_IMAGE_INVALID);
    return 1;
  }

  /* allocate the first BAM buffer if required */
  if (bam_buffer == NULL) {
    if (bam_buffer_alloc(&bam_buffer))
      return 1;
  }

  partition[part].imagetype = imagetype;
  path->dir.dxx.track  = get_param(part, DIR_TRACK);
  path->dir.dxx.sector = get_param(part, DIR_START_SECTOR);

  bam_refcount++;

  if (imagetype & D64_HAS_ERRORINFO)
    /* Invalidate error cache */
    errorcache.part = 255;

  return 0;
}

static uint8_t d64_opendir(dh_t *dh, path_t *path) {
  dh->part = path->part;
  dh->dir.d64.track  = path->dir.dxx.track;
  dh->dir.d64.sector = path->dir.dxx.sector;
  dh->dir.d64.entry  = 0;
  return 0;
}

static int8_t d64_readdir(dh_t *dh, cbmdirent_t *dent) {
  int8_t res;

  do {
    res = nextdirentry(dh);
    if (res)
      return res;

    if (ops_scratch[DIR_OFS_FILE_TYPE] != 0)
      break;
  } while (1);

  memset(dent, 0, sizeof(cbmdirent_t));

  dent->opstype = OPSTYPE_DXX;
  dent->typeflags = ops_scratch[DIR_OFS_FILE_TYPE] ^ FLAG_SPLAT;

  if ((dent->typeflags & TYPE_MASK) > TYPE_DIR)
    /* Change invalid types to DEL */
    /* FIXME: Should exclude DIR on non-native-subdir formats */
    dent->typeflags &= (uint8_t)~TYPE_MASK;

  dent->pvt.dxx.dh = dh->dir.d64;
  dent->pvt.dxx.dh.entry -= 1; /* undo increment in nextdirentry */
  dent->blocksize = ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI];
  dent->remainder = 0xff;
  memcpy(dent->name, ops_scratch + DIR_OFS_FILE_NAME, CBM_NAME_LENGTH);
  strnsubst(dent->name, 16, 0xa0, 0);

  /* sanitize values in case they were not stored */
  dent->date.minute=  ops_scratch[DIR_OFS_MINUTE] % 60;
  dent->date.hour  =  ops_scratch[DIR_OFS_HOUR] % 24;
  dent->date.day   = (ops_scratch[DIR_OFS_DAY]   - 1) % 31 + 1;
  dent->date.month = (ops_scratch[DIR_OFS_MONTH] - 1) % 12 + 1;
  dent->date.year  =  ops_scratch[DIR_OFS_YEAR] % 100;
  /* adjust year into 1980..2079 range */
  if (dent->date.year < 80)
    dent->date.year += 100;

  return 0;
}

/* Reads and converts a string from the dir header sector (BAM for D41) to the buffer */
/* Used by d64_get(disk|dir)label and d64_getid */
static uint8_t read_string_from_dirheader(path_t *path, uint8_t *buffer, param_t what, uint8_t size) {
  uint8_t sector = 0;

  if (image_read(path->part,
                 sector_offset(path->part, path->dir.dxx.track, sector)
                 + get_param(path->part, what),
                 buffer, size))
    return 1;

  strnsubst(buffer, size, 0xa0, 0x20);
  return 0;
}

static uint8_t d64_getdirlabel(path_t *path, uint8_t *label) {
  return read_string_from_dirheader(path, label, LABEL_OFFSET, 16);
}

static uint8_t d64_getdisklabel(uint8_t part, uint8_t *label) {
  path_t curpath;
  curpath.part = part;
  curpath.dir = partition[part].current_dir;
  if (d64_getdirlabel(&curpath, label))
    return 1;

  /* Zero-terminate label */
  uint8_t *ptr = label+16;
  *ptr-- = 0;
  while (ptr != label && *ptr == ' ')
    *ptr-- = 0;

  return 0;
}

static uint8_t d64_getid(path_t *path, uint8_t *id) {
  return read_string_from_dirheader(path, id, ID_OFFSET, 5);
}

static uint16_t d64_freeblocks(uint8_t part) {
  uint16_t blocks = 0;
  uint8_t i;

  for (i = 1; i != 0 && i <= get_param(part, LAST_TRACK); i++) {
    /* Skip directory track */
    if (i == D41_BAM_TRACK)
      continue;
    blocks += sectors_free(part,i);
  }

  return blocks;
}

static void d64_open_read(path_t *path, cbmdirent_t *dent, buffer_t *buf) {
  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return;

  buf->data[0] = ops_scratch[DIR_OFS_TRACK];
  buf->data[1] = ops_scratch[DIR_OFS_SECTOR];

  buf->pvt.d64.part = path->part;

  buf->read    = 1;
  buf->refill  = d64_read;
  buf->seek    = d64_seek;
  stick_buffer(buf);

  buf->refill(buf);
}

static void d64_open_write(path_t *path, cbmdirent_t *dent, uint8_t type, buffer_t *buf, uint8_t append) {
  dh_t dh;
  uint8_t *ptr;

  /* Check for read-only image file */
  if (!(partition[path->part].imagehandle.flag & FA_WRITE)) {
    set_error(ERROR_WRITE_PROTECT);
    return;
  }

  if (append) {
    /* Append case: Open the file and read the last sector */
    d64_open_read(path, dent, buf);
    while (!current_error && buf->data[0])
      buf->refill(buf);

    if (current_error)
      return;

    /* Modify the buffer for writing */
    buf->pvt.d64.dh     = dent->pvt.dxx.dh;
    buf->pvt.d64.blocks = ops_scratch[DIR_OFS_SIZE_LOW] + 256 * ops_scratch[DIR_OFS_SIZE_HI]-1;
    buf->read       = 0;
    buf->position   = buf->lastused+1;
    if (buf->position == 0)
      buf->mustflush = 1;
    else
      buf->mustflush = 0;
    buf->refill     = d64_write;
    buf->cleanup    = d64_write_cleanup;
    buf->seek       = d64_seek;
    mark_write_buffer(buf);

    update_timestamp(ops_scratch);
    write_entry(buf->pvt.d64.part, &buf->pvt.d64.dh, ops_scratch, 1);

    return;
  }

  /* Non-append case */

  /* Search for an empty directory entry */
  if (find_empty_entry(path, &dh))
    return;

  /* Create directory entry in ops_scratch */
  uint8_t *name = dent->name;
  memset(ops_scratch + 2, 0, sizeof(ops_scratch) - 2);  /* Don't overwrite the link pointer! */
  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);
  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*name) *ptr++ = *name++;
  ops_scratch[DIR_OFS_FILE_TYPE] = type;

  /* Find a free sector and allocate it */
  uint8_t t,s;

  if (get_first_sector(path->part,&t,&s))
    return;

  ops_scratch[DIR_OFS_TRACK]  = t;
  ops_scratch[DIR_OFS_SECTOR] = s;

  if (allocate_sector(path->part,t,s))
    return;

  /* Write the directory entry */
  update_timestamp(ops_scratch);
  if (write_entry(path->part, &dh.dir.d64, ops_scratch, 1))
    return;

  /* Prepare the data buffer */
  mark_write_buffer(buf);
  buf->position       = 2;
  buf->lastused       = 2;
  buf->cleanup        = d64_write_cleanup;
  buf->refill         = d64_write;
  buf->seek           = d64_seek;
  buf->data[2]        = 13; /* Verified on VICE */
  buf->pvt.d64.dh     = dh.dir.d64;
  buf->pvt.d64.part   = path->part;
  buf->pvt.d64.track  = t;
  buf->pvt.d64.sector = s;
}

static void d64_open_rel(path_t *path, cbmdirent_t *dent, buffer_t *buf, uint8_t length, uint8_t mode) {
  set_error(ERROR_SYNTAX_UNABLE);
}

static uint8_t d64_delete(path_t *path, cbmdirent_t *dent) {
  uint8_t linkbuf[2];

  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return 255;

  /* Free the sector chain in the BAM */
  linkbuf[0] = ops_scratch[DIR_OFS_TRACK];
  linkbuf[1] = ops_scratch[DIR_OFS_SECTOR];

  do {
    free_sector(path->part, linkbuf[0], linkbuf[1]);

    if (checked_read(path->part, linkbuf[0], linkbuf[1], linkbuf, 2, ERROR_ILLEGAL_TS_LINK))
      return 255;
  } while (linkbuf[0]);

  /* Clear directory entry */
  ops_scratch[DIR_OFS_FILE_TYPE] = 0;
  if (write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1))
    return 255;

  return 1;
}

static void d64_read_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  checked_read(part, track, sector, buf->data, 256, ERROR_ILLEGAL_TS_COMMAND);
}

static void d64_write_sector(buffer_t *buf, uint8_t part, uint8_t track, uint8_t sector) {
  if (track < 1 || track > get_param(part, LAST_TRACK) ||
      sector >= sectors_per_track(part, track)) {
    set_error_ts(ERROR_ILLEGAL_TS_COMMAND,track,sector);
  } else
    image_write(part, sector_offset(part,track,sector), buf->data, 256, 1);
}

static void d64_rename(path_t *path, cbmdirent_t *dent, uint8_t *newname) {
  uint8_t *ptr;

  /* Read the directory entry of the file */
  if (read_entry(path->part, &dent->pvt.dxx.dh, ops_scratch))
    return;

  memset(ops_scratch + DIR_OFS_FILE_NAME, 0xa0, CBM_NAME_LENGTH);
  ptr = ops_scratch + DIR_OFS_FILE_NAME;
  while (*newname) *ptr++ = *newname++;

  write_entry(path->part, &dent->pvt.dxx.dh, ops_scratch, 1);
}


/**
 * d64_raw_directory - open directory track as file
 * @buf: target buffer
 *
 * This function opens the directory track as a file on the
 * buffer passed in buf. Used when $ is opened on a secondary
 * address > 0.
 */
void d64_raw_directory(path_t *path, buffer_t *buf) {
  /* Copy&Waste from d64_open_read */
  buf->data[0] = path->dir.dxx.track;
  buf->data[1] = 0;

  buf->pvt.d64.part = path->part;

  buf->read    = 1;
  buf->refill  = d64_read;
  buf->seek    = d64_seek;
  stick_buffer(buf);

  buf->refill(buf);
}

/**
 * d64_chdir - chdir for Dxx files
 * @path   : path object of the location of dirname
 * @dirname: directory to be changed into
 *
 * Changes the directory in the path object.
 * Returns 0 if successful, 1 otherwise.
 */
static uint8_t d64_chdir(path_t *path, cbmdirent_t *dirname) {
  return image_chdir(path,dirname);
}

/**
 * d64_mkdir - mkdir for Dxx files
 * @path   : path object of the location of dirname
 * @dirname: directory to be created
 *
 * Creates a subdirectory.
 */
static void d64_mkdir(path_t *path, uint8_t *dirname) {
  (void)path;
  (void)dirname;
  set_error(ERROR_SYNTAX_UNABLE);
}

/**
 * d64_invalidate - invalidate internal state
 *
 * This function invalidates all cached state when
 * a card change is detected.
 */
void d64_invalidate(void) {
  free_buffer(bam_buffer);
  bam_buffer   = NULL;
  free_buffer(bam_buffer2);
  bam_buffer2  = NULL;
  bam_refcount = 0;
}

/**
 * d64_unmount - unmount disk image
 * @part: partition number
 *
 * This function in called on image unmount and handles
 * refcounting for the BAM buffers.
 */
void d64_unmount(uint8_t part) {
  /* invalidate BAM buffers that point to the current partition */
  if (bam_buffer) {
    bam_buffer->cleanup(bam_buffer);
    if (bam_buffer->pvt.bam.part == part)
      bam_buffer->pvt.bam.part = 255;
  }

  if (bam_buffer2) {
    bam_buffer2->cleanup(bam_buffer2);
    if (bam_buffer2->pvt.bam.part == part)
      bam_buffer2->pvt.bam.part = 255;
  }

  /* decrease BAM buffer refcounter - it can never be zero while a Dxx is mounted*/
  if (--bam_refcount) {
    free_buffer(bam_buffer);
    free_buffer(bam_buffer2);
    bam_buffer  = NULL;
    bam_buffer2 = NULL;
  }
}


/* ------------------------------------------------------------------------- */
/*  Formatting disk images                                                   */
/* ------------------------------------------------------------------------- */

/* copy name and id into some buffer */
static void format_copy_label(uint8_t part, uint8_t *data, uint8_t *name, uint8_t *idbuf) {
  uint8_t *ptr;
  uint8_t count;

  ptr = data + get_param(part, LABEL_OFFSET);
  memset(ptr, 0xa0, 25); /* 25=name+id+dos+spaces+padding */

  count = 16;
  while (*name && count--)
    *ptr++ = *name++;

  memcpy(data + get_param(part, ID_OFFSET), idbuf, 5);
}

static void format_d41_image(uint8_t part, buffer_t *buf, uint8_t *name, uint8_t *idbuf) {
  /* allocate BAM and first directory sector */
  for (uint8_t s=0; s<2; s++)
    allocate_sector(part, D41_BAM_TRACK, s);

  /* 18/0 is now available via bam_buffer */
  uint8_t *ptr = bam_buffer->data;
  *ptr++ = 18;
  *ptr++ = 1;
  *ptr++ = 0x41;

  /* copy disk label and ID */
  idbuf[3] = '2';
  idbuf[4] = 'A';
  format_copy_label(part, bam_buffer->data, name, idbuf);
  /* two additional 0xa0 characters on 5.25" disks */
  bam_buffer->data[0xa9] = 0xa0;
  bam_buffer->data[0xaa] = 0xa0;

  clear_dir_sector(part, D41_BAM_TRACK, 1, buf->data);
}

static void d64_format(uint8_t part, uint8_t *name, uint8_t *id) {
  buffer_t *buf;
  uint8_t  idbuf[5];
  uint16_t t;
  uint16_t  s;

  /* grab a buffer as work area */
  buf = alloc_buffer();
  if (buf == NULL)
    return;

  mark_write_buffer(buf);
  unstick_buffer(buf);
  mark_buffer_dirty(buf);
  memset(buf->data, 0, 256);

  /* Flush BAM buffers and mark their contents as invalid */
  d64_bam_commit();
  bam_buffer->pvt.bam.part = 0xff;
  if (bam_buffer2)
    bam_buffer2->pvt.bam.part = 0xff;

  if (id != NULL) {
    /* Clear the data area of the disk image */
    for (t=1; t<=get_param(part, LAST_TRACK); t++) {
      for (s=0; s<sectors_per_track(part, t); s++) {
        if (image_write(part, sector_offset(part, t, s),
                        buf->data, 256, 0))
          return;
      }
    }

    /* Copy the new ID into the buffer */
    idbuf[0] = id[0];
    idbuf[1] = id[1];
  } else {
    /* Read the old ID into the buffer */
    path_t path;
    path.part = part;
    path.dir.dxx.track  = get_param(part, DIR_TRACK);
    path.dir.dxx.sector = 0;
    if (d64_getid(&path, idbuf))
      return;

    /* clear the entire directory track */
    /* This is not accurate, but I do not care. */
    t = get_param(part, DIR_TRACK);
    for (s=0; s < sectors_per_track(part, t); s++) {
      if (image_write(part, sector_offset(part, t, s),
                      buf->data, 256, 0))
        return;
    }
  }
  idbuf[2] = 0xa0;

  /* Mark all sectors as free */
  for (t=1; t<=get_param(part, LAST_TRACK); t++) {
    for (s=0; s<sectors_per_track(part, t); s++)
      free_sector(part,t,s);
  }

  /* call imagetype-specific format function */
  partition[part].d64data.format_function(part, buf, name, idbuf);

  /* FIXME: Clear the error info block */
}


/* ------------------------------------------------------------------------- */
/*  ops struct                                                               */
/* ------------------------------------------------------------------------- */

const PROGMEM fileops_t d64ops = {
  d64_open_read,
  d64_open_write,
  d64_open_rel,
  d64_delete,
  d64_getdisklabel,
  d64_getdirlabel,
  d64_getid,
  d64_freeblocks,
  d64_read_sector,
  d64_write_sector,
  d64_format,
  d64_opendir,
  d64_readdir,
  d64_mkdir,
  d64_chdir,
  d64_rename
};
