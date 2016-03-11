#!/bin/bash
# Stock Samsung kernel for Samsung Exynos 8890 devices build script by jcadduono

################### BEFORE STARTING ################
#
# download a working toolchain and extract it somewhere and configure this
# file to point to the toolchain's root directory.
#
# once you've set up the config section how you like it, you can simply run
# ./build.sh [VARIANT]
#
###################### DEVICES #####################
#
# herolte = Galaxy S7
#
# xx   = International & Duos, Canada
#        SM-G930F, SM-G930FD, SM-G930X, SM-G930W8
#
# kor  = Korea
#        SM-G930K, SM-G930L, SM-G930S
#
# hero2lte = Galaxy S7 edge
#
# xx   = International & Duos, Canada
#        SM-G935F, SM-G935FD, SM-G935X, SM-G935W8
#
# kor  = Korea
#        SM-G935K, SM-G935L, SM-G935S
#
# gracelte = Galaxy Note 7
#
# xx   = International & Duos
#        SM-N930F, SM-N930FD, SM-N930X
#
# kor  = Korea
#        SM-N930K, SM-N930L, SM-N930S
#
###################### CONFIG ######################

# root directory of universal8890 kernel git repo (default is this script's location)
RDIR=$(pwd)

[ "$VER" ] ||
# version number
VER=$(cat "$RDIR/VERSION")

# directory containing cross-compile arm64 toolchain
TOOLCHAIN=$HOME/build/toolchain/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu

CPU_THREADS=$(grep -c "processor" /proc/cpuinfo)
# amount of cpu threads to use in kernel make process
THREADS=$((CPU_THREADS + 1))

############## SCARY NO-TOUCHY STUFF ###############

ABORT()
{
	[ "$1" ] && echo "Error: $*"
	exit 1
}

export ARCH=arm64
export CROSS_COMPILE=$TOOLCHAIN/bin/aarch64-linux-gnu-

[ -x "${CROSS_COMPILE}gcc" ] ||
ABORT "Unable to find gcc cross-compiler at location: ${CROSS_COMPILE}gcc"

[ "$TARGET" ] || TARGET=samsung
[ "$1" ] && DEVICE=$1
[ "$2" ] && VARIANT=$2
[ "$DEVICE" ] || DEVICE=gracelte
[ "$VARIANT" ] || VARIANT=xx

DEFCONFIG=${TARGET}_defconfig
DEVICE_DEFCONFIG=device_${DEVICE}_${VARIANT}

[ -f "$RDIR/arch/$ARCH/configs/${DEFCONFIG}" ] ||
ABORT "Config $DEFCONFIG not found in $ARCH configs!"

[ -f "$RDIR/arch/$ARCH/configs/${DEVICE_DEFCONFIG}" ] ||
ABORT "Device config $DEVICE_DEFCONFIG not found in $ARCH configs!"

export LOCALVERSION=$TARGET-$DEVICE-$VARIANT-$VER

CLEAN_BUILD()
{
	echo "Cleaning build..."
	rm -rf build
}

SETUP_BUILD()
{
	echo "Creating kernel config for $LOCALVERSION..."
	mkdir -p build
	make -C "$RDIR" O=build "$DEFCONFIG" \
		DEVICE_DEFCONFIG="$DEVICE_DEFCONFIG" \
		|| ABORT "Failed to set up build"
}

BUILD_KERNEL()
{
	echo "Starting build for $LOCALVERSION..."
	while ! make -C "$RDIR" O=build -j"$THREADS"; do
		read -rp "Build failed. Retry? " do_retry
		case $do_retry in
			Y|y) continue ;;
			*) return 1 ;;
		esac
	done
}

INSTALL_MODULES() {
	echo "Installing kernel modules to build/lib/modules..."
	make -C "$RDIR" O=build \
		INSTALL_MOD_PATH="." \
		INSTALL_MOD_STRIP=1 \
		modules_install
	rm build/lib/modules/*/build build/lib/modules/*/source
}

BUILD_DTB()
{
	echo "Generating dtb.img..."
	./dtbgen.sh "$DEVICE" "$VARIANT" || ABORT "Failed to generate dtb.img!"
}

cd "$RDIR" || ABORT "Failed to enter $RDIR!"

CLEAN_BUILD &&
SETUP_BUILD &&
BUILD_KERNEL &&
INSTALL_MODULES &&
BUILD_DTB &&
echo "Finished building $LOCALVERSION!"
