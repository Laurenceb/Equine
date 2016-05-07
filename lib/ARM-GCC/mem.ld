/* Linker script to configure memory regions. 
 * Need modifying for a specific board. 
 *   FLASH.ORIGIN: starting address of flash
 *   FLASH.LENGTH: length of flash
 *   RAM.ORIGIN: starting address of RAM bank 0
 *   RAM.LENGTH: length of RAM bank 0
 */
MEMORY
{
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x5000/*20K*/
  FLASH (rx) : ORIGIN = 0x8000000, LENGTH = 0x20000/*128K*/
}

__HEAP_SIZE = 0x2000; /*8K heap size*/
__STACK_SIZE = 0xc00; /*3K stack size*/
