#!/usr/bin/env bash
# Build this tree for the ASUS Vivobook S15 (S5507QA, X1E80100).
#
#   vivobook-s15/build.sh                                  native build on the laptop
#   CROSS_COMPILE=aarch64-linux-gnu- vivobook-s15/build.sh cross build on x86-64
#
# Environment: O (build dir, default ./build-vivobook), OUT (staging dir, default $O/out),
# JOBS (default nproc). Needs make, gcc, flex, bison, bc, openssl and libelf headers,
# dtc + fdtoverlay, python3, depmod (kmod).
#
# Result in $OUT:
#   vmlinuz-<ver>                         arm64 Image (EFI stub)
#   x1e80100-asus-vivobook-s15-el2.dtb    the EL2 DTB with dt-overlays/*.dtso applied
#   rootfs/lib/modules/<ver>/             in-tree modules + extra/ (external-modules)
#   config-<ver>, System.map-<ver>
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
SRC=$(dirname "$here")
O=${O:-$SRC/build-vivobook}
OUT=${OUT:-$O/out}
JOBS=${JOBS:-$(nproc)}
DTB=qcom/x1e80100-asus-vivobook-s15-el2.dtb
EXT=(adsp-sleepstate adsp-pwr-lmts aoss-cxsd-only)
MAKE=(make -C "$SRC" O="$O" -j"$JOBS" ARCH=arm64 ${CROSS_COMPILE:+CROSS_COMPILE=$CROSS_COMPILE} DTC_FLAGS=-@)

mkdir -p "$O" "$OUT"

# 1. Config: the saved one, new symbols at their defaults
cp "$here/config" "$O/.config"
"${MAKE[@]}" olddefconfig
KREL=$("${MAKE[@]}" -s kernelrelease)
echo "== building $KREL"

# 2. Kernel, modules, device trees
"${MAKE[@]}" Image modules dtbs

# 3. Out-of-tree modules (also packaged for DKMS: external-modules/*/dkms.conf)
for m in "${EXT[@]}"; do
	rm -rf "$O/ext-$m"
	cp -r "$here/external-modules/$m" "$O/ext-$m"
	"${MAKE[@]}" M="$O/ext-$m" modules
done

# 4. Staging
rm -rf "$OUT/rootfs"
"${MAKE[@]}" INSTALL_MOD_PATH="$OUT/rootfs" INSTALL_MOD_STRIP=1 modules_install
mkdir -p "$OUT/rootfs/lib/modules/$KREL/extra"
for m in "${EXT[@]}"; do
	install -m644 "$O/ext-$m"/*.ko "$OUT/rootfs/lib/modules/$KREL/extra/"
done
depmod -b "$OUT/rootfs" "$KREL"

cp "$O/arch/arm64/boot/Image" "$OUT/vmlinuz-$KREL"
cp "$O/.config" "$OUT/config-$KREL"
cp "$O/System.map" "$OUT/System.map-$KREL"

# 5. Device tree + overlays
ovl=()
for f in "$here"/dt-overlays/*.dtso; do
	o=$O/$(basename "${f%.dtso}").dtbo
	dtc -q -@ -I dts -O dtb -o "$o" "$f"
	ovl+=("$o")
done
fdtoverlay -i "$O/arch/arm64/boot/dts/$DTB" -o "$OUT/$(basename "$DTB")" "${ovl[@]}"

echo "== done: $OUT"
ls -la "$OUT"
