/* Linker script for the GD32C1x3 series, which all have the same flash/ram layout */
MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM : ORIGIN = 0x20000000, LENGTH = 32K
}