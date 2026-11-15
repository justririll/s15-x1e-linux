#!/bin/sh
# slbounce v5 (TravMurav): systemd-boot EFI driver that switches to EL2 when the entry's DTB
# is an "-el2" one. Needs tcblaunch.exe (from Windows, kept next to this script) at the ESP root.
set -eu
url=https://github.com/TravMurav/slbounce/releases/download/v5/slbounce.efi
sum=96c89a06a139b9e39cef9f52896a1a2c8cd5437d0feeefb2a5c7ea5b35614e17
out=${1:-slbounceaa64.efi}
curl -fL -o "$out" "$url"
echo "$sum  $out" | sha256sum -c -
