# FreeBSD/RISC-V
This is a port of [FreeBSD Operating System](http://www.freebsd.org) to RISC-V instruction set architecture.

[![Build Status](https://ci.freebsd.org/buildStatus/icon?job=FreeBSD-head-riscv64-build)](https://ci.freebsd.org/job/FreeBSD-head-riscv64-build/)

### Prepare your environment
On FreeBSD 11.0 machine install the required packages:
```
$ sudo pkg install riscv64-xtoolchain-gcc qemu-riscv riscv-isa-sim
```

## Quick way

### You can use pre-built images, otherwise proceed to next step.
```
fetch https://artifact.ci.freebsd.org/snapshot/head/latest/riscv/riscv64/bbl-spike.xz
unxz bbl-spike.xz
```

## Complete build from scratch
Set the following environment variables:
```
$ setenv MAKEOBJDIRPREFIX /home/${USER}/obj/
$ setenv WITHOUT_FORMAT_EXTENSIONS yes
$ setenv DESTDIR /home/${USER}/riscv-world
```

### Build FreeBSD world
```
$ svnlite co http://svn.freebsd.org/base/head freebsd-riscv
$ cd freebsd-riscv
$ make -j4 CROSS_TOOLCHAIN=riscv64-gcc TARGET_ARCH=riscv64 buildworld
```

### Build 32mb rootfs image
```
$ cd freebsd-riscv
$ make TARGET_ARCH=riscv64 -DNO_ROOT -DWITHOUT_TESTS DESTDIR=$DESTDIR installworld
$ make TARGET_ARCH=riscv64 -DNO_ROOT -DWITHOUT_TESTS DESTDIR=$DESTDIR distribution
$ fetch https://raw.githubusercontent.com/bukinr/riscv-tools/master/image/basic.files
$ tools/tools/makeroot/makeroot.sh -s 32m -f basic.files riscv.img $DESTDIR
```

### Prepare your kernel config
Modify sys/riscv/conf/GENERIC. Uncomment the following lines and specify the path to your riscv.img:
```
options 	MD_ROOT
options 	MD_ROOT_SIZE=32768	# 32MB ram disk
makeoptions	MFS_IMAGE=/path/to/riscv.img
options 	ROOTDEVNAME=\"ufs:/dev/md0\"
```

### Build FreeBSD kernel
```
$ cd freebsd-riscv
for Spike:
$ make -j4 CROSS_TOOLCHAIN=riscv64-gcc TARGET_ARCH=riscv64 KERNCONF=SPIKE buildkernel
for QEMU:
$ make -j4 CROSS_TOOLCHAIN=riscv64-gcc TARGET_ARCH=riscv64 KERNCONF=QEMU buildkernel
```

### Build BBL
```
$ git clone https://github.com/freebsd-riscv/riscv-pk
$ cd riscv-pk
$ mkdir build && cd build
$ setenv CFLAGS "-nostdlib"
$ ../configure --host=riscv64-unknown-freebsd11.0 --with-payload=path_to_freebsd_kernel
$ gmake LIBS=''
$ unsetenv CFLAGS
```

### Run Spike simulator
```
$ spike -m1024 -p2 /path/to/bbl
```

### Run QEMU emulator
```
$ qemu-system-riscv64 -m 2048M -kernel /path/to/bbl -nographic
```

Additional information is available on [FreeBSD Wiki](http://wiki.freebsd.org/riscv).
