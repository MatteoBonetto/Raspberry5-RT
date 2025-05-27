# PREEMPT-RT on Raspbian OS and Raspberry 5

This repo explains how to recompile Linux kernel for Raspbian OS and enable the Preempt-RT scheduling mechanism.

It also implements a reliable C++17, header-only, templated `Timer` class.

## Enable the RT scheduler

**Note**: this step requires re-compiling the Linux kernel. On a Raspberry 5 it typically takes a couple of hours.

### Prepare the environment

Starting with Linux kernel 6.12, the `RT_patch` has been merged in the canonical source tree, so there is no need to patch anymore.

```sh
sudo apt install git bc bison flex libssl-dev make
sudo apt install libncurses5-dev
sudo apt install raspberrypi-kernel-headers

mkdir ~/kernel
cd kernel
git clone --depth 1 --branch rpi-6.15.y https://github.com/raspberrypi/linux
```

### Configure for the Raspberry 5

This may be different for a Rpi 4 (I did not have one at hand to test).

```sh
make bcm2712_defconfig
```

### Enable the `PREEMPT_RT` scheduler

```sh
make menuconfig
```

In the interactive screen that follows, go to *General Setup/Preemption Model/Fully Preemptible Kernel (Real-Time)* and enable it. Save and exit.

### Compile the kernel

This may take about an hour:

```sh
make prepare
make CFLAGS='-O3 -march=native' -j6 Image.gz modules dtbs # recommendation is 1.5 times the number of cores (=4), which equals 6
sudo make -j6 modules_install 
```

### Install kernel and modules

``` sh
sudo mkdir /boot/firmware/RT
sudo mkdir /boot/firmware/RT/overlays-RT
sudo cp arch/arm64/boot/dts/broadcom/*.dtb /boot/firmware/RT/
sudo cp arch/arm64/boot/dts/overlays/*.dtb* /boot/firmware/RT/overlays-RT/
sudo cp arch/arm64/boot/dts/overlays/README /boot/firmware/RT/overlays-RT
sudo cp arch/arm64/boot/Image.gz /boot/firmware/kernel_2712-RT.img
```

### Enable the new kernel

Append the following lines **at the end** of `/boot/firmware/config.txt`:

```
os_prefix=RT/
overlay_prefix=overlays-RT/
kernel=/kernel_2712-RT.img
``` 

Then `sudo reboot` to load the new kernel.

### Update the Raspberry firmware

```sh
sudo SKIP_KERNEL=1 PRUNE_MODULES=1 rpi-update rpi-6.15.y
```

Verify that it worked:

```sh
uname -a
````

It must return: 

```
Linux mads-pi 6.15.0-rc7-v8-16k+ #1 SMP PREEMPT_RT Mon May 26 19:13:33 CEST 2025 aarch64 GNU/Linux
```

Note the `PREEMPT_RT` key.


### How to replicate

To install on another Raspberry, probably the simplest way is to archive and copy the `linux` folder:

```sh
cd ..
tar czvf linux_6.15_RT_RPi5.tgz linux
```


## The `Timer` class

The `Timer` class provides a precise timing mechanism. Usage is simple

First instantiate the timer. It is a templated class that takes two parameters:

* first template parameter defines the `std::chrono` duration type. For example, `duration<double> d(1.5)` means 1.5 seconds, `milliseconds d(200)` means 200 milliseconds.
* second template parameter, optional, decides to enable the running 
statistics facility. For normal operations you want to keep it off to 
spare computation time

```cpp
using namespace std;
using namespace chrono;
// ...
milliseconds d(100);
milliseconds max_d(200); 
Timer<milliseconds, true> t(d, max_d);
t.enable_rt_scheduler(); // only on PREEMPT_RT kernels!
t.start();
```

Then, within the timed loop, call `t.wait()` to pause the loop until the next multiple of the requested time step. This function returns -1 if the timing could not be respectd and the `max_d` time has elapsed.

Alternatively, you can call `t.wait_throw()`, which throws a runtime error if the loop took more than `max_d`.

The timer can be disable with `t.stop()`, and running statistics can be obtained with `t.stats()`.

### Building project example

On a standard kernel:

```sh
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release
cmake --build build 
```

On a PREEMPT_RT kernel:

```sh
cmake -Bbuild -DCMAKE_BUILD_TYPE=Release -DENABLE_RT_SCHEDULER=ON
cmake --build build
```

Then run it as:

```sh
sudo build/timer [interval in seconds with decimals] > [destination_log.csv]
```

Note that if you enable the RT with `t.enable_rt_scheduler()`, then you must launch it as `sudo`.
