# RFFE Controller Firmware

Firmware for the RFFE Control Boards, based on MBED, using a Cortex M3 LPC1768 processor.

## Pre-requisites

The following packages must be installed on your system in order to compile the firmware:
- **gcc-arm-none-eabi**
- **cmake**
- **cmake-gui** (Optional)

**gcc-arm-none-eabi** can be installed from the pre-compiled files found at: https://launchpad.net/gcc-arm-embedded/+download
or you can run the following command under Ubuntu:

    sudo apt-get install gcc-arm-none-eabi

Next step is to clone this repository into your workspace. Since we're using the mbed libraries as a submodule, you **MUST** run the git clone command with the `--recursive` option.

	git clone --recursive https://github.com/lnls-dig/rffe-fw

If you've already cloned the repository without the recursive option, go to the source folder and run:

	cd mbed-libs
	git submodule update --init --recursive

## Compilation

Go to the repository folder

    cd /path/to/repo/

If this is the first time compiling this firmware, run CMake configuration scripts

	cmake .

You can set several options (Board IP, DHCP Mode, Mount path, libs to compile, etc) in `CMakeCache.txt` file using your text editor or a GUI editor for CMake (`cmake-gui`).
After changing the desired options, run the CMake configuration command again (the CMakeCache file will not be edited by this command) and compile the firmware:

	make -s

*NOTE: The compiler will return several warnings, most of them are regarding the mbed libraries, but since they have a stable version on github, we'll just use the master branch and ignore those warnings.*

Both a `.axf` file and a `.bin` file will be generated in the source folder. You can use any one you prefer to program your processor.

To clean the compilation files (binaries, objects and dependence files), just run

    make clean

## Programming
To program the firmware in the MBED board, just plug in a USB cable in its frontal jack in your computer and a `MBED` drive will be mounted (the MBED will get its power from the USB +5v).

Copy the generated binary file before into the MBED storage and reset the board (Power Cycle or Reset button).

**NOTE:**The MBED bootloader will run only the newest binary file found in its drive, therefore, you can have multiple revisions of the firmware stored for backup purposes.
