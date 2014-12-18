/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x0003A000, LENGTH = 0x5000 /* Bootloader on the last 12K of flash. before the MBS 4K */
  RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x2000 /* 8 kB is taken by S110,8 kB for app. */
}

INCLUDE "gcc_nrf51_common.ld"
