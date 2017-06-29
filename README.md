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

Create a new folder wherever is suitable

	cd <build_folder>

Run CMake using the path to the repository folder as an direct argument and the flags configure the compilation scripts to your specific build. It's possible to set several options like Board IP, DHCP Mode, Mount path, libs to compile, etc in `CMakeCache.txt` file using your text editor, passing the options via `-D<option>=<value>` or a GUI editor for CMake (`cmake-gui`).

        cmake <path_to_source> -D<option1>=<value1> -D<option2>=<value2> ...

Example:

	cmake ../ -DUSE_FIXIP=true -DMBED_IP=10.2.119.203 -DMBED_MASK=10.2.119.255 -DMBED_GATEWAY=10.2.119.1

After changing the desired options, run the CMake configuration command again, if `CMakeCache.txt` was changed, and compile the firmware:

	make -s

*NOTE: The compiler will return several warnings, most of them are regarding the mbed libraries, but since they have a stable version on github, we'll just use the master branch and ignore those warnings.*

Both a `.axf` file and a `.bin` file will be generated in the source folder. You can use any one you prefer to program your processor.

To clean the compilation files (binaries, objects and dependence files), just run

	make clean

## Programming

To program the firmware in the MBED board, just plug in a USB cable in its frontal jack in your computer and a `MBED` drive will be mounted (the MBED will get its power from the USB +5v).

Copy the generated binary file before into the MBED storage and reset the board (Power Cycle or Reset button).

**NOTE:**The MBED bootloader will run only the newest binary file found in its drive, therefore, you can have multiple revisions of the firmware stored for backup purposes.

## Logging

A useful procedure if to log the serial messages from the RFFE via serial cable for debugging purposes. In order to do that, one can use the following logrotate configration file, so to keep long-running logs in a machine:

```
cat <<-EOF > /etc/logrotate.d/rffe-debug
	/var/log/rffe-debug/*.log {
	    hourly
	    rotate 20
	    size 100K
	    maxsize 1M
	    compress
	    missingok
	    notifempty
	    postrotate
	    /bin/kill -HUP `cat /run/rffe-debug.pid 2>/dev/null` 2> /dev/null || true
	    (sudo screen -c /var/log/rffe-debug/screenrc -L /dev/ttyACM0 115200) & PID=$(echo $!) && sudo bash -c "touch /run/rffe-debug.pid  && echo ${PID} > /run/rffe-debug.pid"
	    endscript
	}
EOF
```

Just copy the above commands into a terminal and set logrotate to run at a resonable time, like 1 minute so as to not keep logs too larger than the limit.

Remember to change the TTY port to the correct one when using this. Most of the time it will by ttyACM0, but it's not guaranteed.

As a last reminder. The procedure will be more useful if the RFFE is set to use debug flags. Fow now, just define the symbol DEBUG_PRINTF at the beggining of main.cpp file:

```c
#define DEBUG_PRINTF

```
