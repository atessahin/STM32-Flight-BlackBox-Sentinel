#include "ff.h"
#include "diskio.h"
#include "sd_driver_init.h"
#include "sd_driver_read.h"
#include "sd_driver_write.h"

static volatile DSTATUS Stat = STA_NOINIT;

DSTATUS disk_status (BYTE pdrv)
{
	//fatfs ask after each process
	//if stat has a STA_NOINIT return FR_NOT_READY
    return Stat;
}

DSTATUS disk_initialize (BYTE pdrv)
{
	//if sd_card already wake up we dont restart again direct ready
    if ((Stat & STA_NOINIT) == 0) {
        return Stat;
    }

    //sd card firt start need to reset
    if (sd_card_reset(0, 0) == SD_OK) {
        //raise the flag
        Stat &= ~STA_NOINIT;
        return Stat; //0
    }

    return STA_NOINIT;
}

DRESULT disk_read (BYTE pdrv, BYTE* buff, LBA_t sector, UINT count)
{

    uint32_t addr = (sd_card_status.capacity == STANDART) ? (sector * 512) : sector;

    if (sd_card_read_multiple_data(0, addr, buff, 512, count) == SD_OK) return RES_OK;
    return RES_ERROR;
}

DRESULT disk_write (BYTE pdrv, const BYTE* buff, LBA_t sector, UINT count)
{
    uint32_t addr = (sd_card_status.capacity == STANDART) ? (sector * 512) : sector;

    if (sd_card_write_multiple_data(0, addr, buff, 512, count) == SD_OK) return RES_OK;
    return RES_ERROR;
}

DRESULT disk_ioctl (BYTE pdrv, BYTE cmd, void* buff)
{
    if (cmd == GET_SECTOR_COUNT) {
        *(DWORD*)buff = 7500000;
        return RES_OK;
    }
    if (cmd == GET_SECTOR_SIZE) {
        *(WORD*)buff = 512;
        return RES_OK;
    }
    if (cmd == GET_BLOCK_SIZE) {
        *(DWORD*)buff = 1;
        return RES_OK;
    }
    if (cmd == CTRL_SYNC) {
        return RES_OK;
    }
    return RES_PARERR;
}

DWORD get_fattime(void)
{
    return  ((DWORD)(2024 - 1980) << 25)
          | ((DWORD)1 << 21)
          | ((DWORD)1 << 16)
          | ((DWORD)0 << 11)
          | ((DWORD)0 << 5)
          | ((DWORD)0 >> 1);
}
