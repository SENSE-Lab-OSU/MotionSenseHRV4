/**
 * @file		nand_ftl_diskio.h
 * @author		Andrew Loebs
 * @brief		Header file of the nand ftl diskio module
 *
 * Glue layer between fatfs diskio and dhara ftl.
 *
 */

#ifndef __NAND_FTL_DISKIO_H
#define __NAND_FTL_DISKIO_H


int nand_ftl_diskio_initialize(void);
int nand_ftl_diskio_status(void);
int nand_ftl_diskio_read(BYTE *buff, LBA_t sector, UINT count);
int nand_ftl_diskio_write(const BYTE *buff, LBA_t sector, UINT count);
int nand_ftl_diskio_ioctl(BYTE cmd, void *buff);

#endif // __NAND_FTL_DISKIO_H
