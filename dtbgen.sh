#!/bin/bash
# simple bash script for generating dtb image

# root directory of universal8890 kernel git repo (default is this script's location)
RDIR=$(pwd)

# directory containing cross-compile arm64 toolchain
TOOLCHAIN=$HOME/build/toolchain/gcc-linaro-5.3.1-2016.05-x86_64_aarch64-linux-gnu

# device dependant variables
PAGE_SIZE=2048
DTB_PADDING=0

export ARCH=arm64
export CROSS_COMPILE=$TOOLCHAIN/bin/aarch64-linux-gnu-

BDIR=$RDIR/build
OUTDIR=$BDIR/arch/$ARCH/boot
DTSDIR=$RDIR/arch/$ARCH/boot/dts
DTBDIR=$OUTDIR/dtb
DTCTOOL=$BDIR/scripts/dtc/dtc
INCDIR=$RDIR/include

ABORT()
{
	[ "$1" ] && echo "Error: $*"
	exit 1
}

cd "$RDIR" || ABORT "Failed to enter $RDIR!"

[ -x "$DTCTOOL" ] ||
ABORT "You need to run ./build.sh first!"

[ -x "${CROSS_COMPILE}gcc" ] ||
ABORT "Unable to find gcc cross-compiler at location: ${CROSS_COMPILE}gcc"

[ "$1" ] && DEVICE=$1
[ "$2" ] && VARIANT=$2
[ "$DEVICE" ] || DEVICE=gracelte
[ "$VARIANT" ] || VARIANT=xx

case $DEVICE in
herolte)
	case $VARIANT in
	can|eur|xx|duos)
		DTSFILES="exynos8890-herolte_eur_open_00 exynos8890-herolte_eur_open_01
				exynos8890-herolte_eur_open_02 exynos8890-herolte_eur_open_03
				exynos8890-herolte_eur_open_04 exynos8890-herolte_eur_open_08
				exynos8890-herolte_eur_open_09"
		;;
	kor|skt|ktt|lgt)
		DTSFILES="exynos8890-herolte_kor_all_00 exynos8890-herolte_kor_all_01
				exynos8890-herolte_kor_all_02 exynos8890-herolte_kor_all_03
				exynos8890-herolte_kor_all_04 exynos8890-herolte_kor_all_08"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
hero2lte)
	case $VARIANT in
	can|eur|xx|duos)
		DTSFILES="exynos8890-hero2lte_eur_open_00 exynos8890-hero2lte_eur_open_01
				exynos8890-hero2lte_eur_open_03 exynos8890-hero2lte_eur_open_04
				exynos8890-hero2lte_eur_open_08"
		;;
	kor|skt|ktt|lgt)
		DTSFILES="exynos8890-hero2lte_kor_all_00 exynos8890-hero2lte_kor_all_01
				exynos8890-hero2lte_kor_all_03 exynos8890-hero2lte_kor_all_04
				exynos8890-hero2lte_kor_all_08"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
gracelte)
	case $VARIANT in
	eur|xx|duos)
		DTSFILES="exynos8890-gracelte_eur_open_00 exynos8890-gracelte_eur_open_01
				exynos8890-gracelte_eur_open_02 exynos8890-gracelte_eur_open_03
				exynos8890-gracelte_eur_open_05 exynos8890-gracelte_eur_open_07
				exynos8890-gracelte_eur_open_09 exynos8890-gracelte_eur_open_11"
		;;
	kor|skt|ktt|lgt)
		DTSFILES="exynos8890-gracelte_kor_all_01 exynos8890-gracelte_kor_all_02
				exynos8890-gracelte_kor_all_03 exynos8890-gracelte_kor_all_05
				exynos8890-gracelte_kor_all_07 exynos8890-gracelte_kor_all_09
				exynos8890-gracelte_kor_all_11 exynos8890-gracelte_kor_all_12"
		;;
	*) ABORT "Unknown variant of $DEVICE: $VARIANT" ;;
	esac
	DTBH_PLATFORM_CODE=0x50a6
	DTBH_SUBTYPE_CODE=0x217584da
	;;
*) ABORT "Unknown device: $DEVICE" ;;
esac

mkdir -p "$OUTDIR" "$DTBDIR"

rm -f "$DTBDIR/"*

echo "Processing dts files..."

for dts in $DTSFILES; do
	echo "=> Processing: ${dts}.dts"
	"${CROSS_COMPILE}cpp" -nostdinc -undef -x assembler-with-cpp -I "$INCDIR" "$DTSDIR/${dts}.dts" > "$DTBDIR/${dts}.dts"
	echo "=> Generating: ${dts}.dtb"
	$DTCTOOL -p $DTB_PADDING -i "$DTSDIR" -O dtb -o "$DTBDIR/${dts}.dtb" "$DTBDIR/${dts}.dts"
done

echo "Generating dtb.img..."
scripts/dtbTool/dtbTool -o "$OUTDIR/dtb.img" -d "$DTBDIR/" -s $PAGE_SIZE --platform $DTBH_PLATFORM_CODE --subtype $DTBH_SUBTYPE_CODE || exit 1

echo "Done."
