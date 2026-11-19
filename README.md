Do not use this unless you know what you are doing. This is wip and may do bad stuff to your sd card if misused.

# emuMMC
*A SDMMC driver replacement for Nintendo's Filesystem Services, by **m4xw***  

### Supported Horizon Versions
**1.0.0 - 23.5.0**

The DDR200 branch uses Hekate's direct Atmosphère launch. See
[DDR200.md](DDR200.md) for the KIPM build and installation steps.

## DDR200 fork credits

- **Dominatorul** develops and maintains this fork.
- **CTCaer** wrote the original DDR200 implementation in Hekate. [Original implementation](https://github.com/CTCaer/hekate/blob/8e1392df4569ac501a115867056b13c361429e9e/bdk/storage/sdmmc.c#L1388)
- **Lightos** wrote the first DDR200 implementation for Horizon OS. [Original fork](https://github.com/Lightos1/emuMMC_ddr200)
- **HAZEL** helped with development and debugging.

## Features
* Arbitrary SDMMC backend selection  
  **This allows loading eMMC from SD or even SD from eMMC**
* On the fly hooking / patching, fully self-infesting  
  **Only one payload required for all versions!**
* File-based SDMMC backend support (from SD)  
  **This allows loading eMMC images from hekate-backups (split or not)**
* SDMMC device based sector offset (*currently eMMC only*)  
  **Raw partition support for eMMC from SD with less performance overhead**
* Full support for `/Nintendo` folder redirection to a arbitrary path  
  **No 8 char length restriction!**
* exosphere based context configuration  
  **This includes full support for multiple emuMMC images**

## Compiling
### hekate
Run `./build.sh` and copy the produced kipm (Kernel Initial Process Modification) file to `/bootloader/sys/`

### Atmosphere
Run `make`, the resulting kip can be used for code injection via fusee (place at `/atmosphere/emummc.kip`)

## License
**emuMMC is released as GPLv2**

## Credits
* **CTCaer** - The CTCaer hekate fork, file-based emuMMC support, SDMMC driver fixes among other things
* **SciresM, hexkyz** - The Atmosphere project, FS offsets, additional research related to newer FS versions
* **naehrwert** - The hekate project, its SDMMC driver and being very helpful in the early research phase
* **jakibaki** - KIP Inject PoC, used in the early dev phase
* **switchbrew/devkitPro** - devkitA64 and libnx sources
