# Linux for the ASUS Vivobook S15 (S5507QA, Snapdragon X Elite X1E80100)

> [!WARNING]
> **This is vibe-coded.** Most of the patches on this branch were written or
> rebased with an AI assistant and tested on exactly one laptop. It's risky,
> but it seems to work. **Use it at your own risk.**
>
> **Your speakers are the main risk.** The WSA8845 speaker amplifiers are
> driven through a DSP speaker-protection path (V/I feedback, R0/T0
> calibration) that this branch sets up itself. If that path is
> misconfigured, or you use a topology without speaker protection, the
> speakers run unprotected and **can be damaged** at high volume. Keep the
> volume sane until you have checked that protection is active (see
> [Speakers](#speakers)).

This is a fork of [linux-msm/laptops-kernel](https://github.com/linux-msm/laptops-kernel).
The branch `vivobook-s15-7.4` is:

- the `qcom-laptops` base at `98172d02f` (**7.4.0-rc1**);
- 92 patches for this laptop, one commit each (list below);
- `vivobook-s15/`, with the kernel config, DT overlays, three out-of-tree
  modules, boot parameters and a build script.

The laptop boots at **EL2** (through [slbounce](https://github.com/TravMurav/slbounce)),
so Linux itself runs the ADSP and CDSP and KVM works.

## Status on 7.4.0-rc1

Tested on 2026-11-15 on Gentoo (arm64, systemd), BIOS S5507QAD.368.

| | Feature | Notes |
|---|---|---|
| ✅ | Boot at EL2 | slbounce + `-el2` DTB, KVM in VHE mode |
| ✅ | Internal display (eDP 2880×1620) | `msm` DPU, console on the panel |
| ⚠️ | GPU (Adreno X1-85) | driver binds and loads the GMU firmware; 3D rendering not tested on 7.4 |
| ✅ | NVMe | |
| ✅ | Wi-Fi (WCN7850, ath12k) | |
| ⚠️ | Bluetooth (WCN7850) | `hci0` comes up and loads its firmware (needs patch 0092); pairing not tested |
| ✅ | Keyboard, touchpad | I2C HID; Fn keys through the EC HID driver |
| ✅ | Embedded controller | fan speed, fan profiles (whisper / standard / performance / full), keyboard RGB, SoC temperature fed to the EC |
| ✅ | Per-profile CPU limits | e.g. whisper caps the CPUs at 998 MHz, performance at 3.4 GHz |
| ✅ | Battery, charge limit | `charge_control_end_threshold` = 80, kept across ADSP restarts |
| ✅ | ADSP / CDSP | started by Linux at EL2 |
| ✅ | Hardware video decode/encode (Iris) | H.264/HEVC/VP9/AV1 decoder, encoder; H.264 encode → decode round trip checked frame by frame |
| ⚠️ | Sound | card registers, speaker calibration loads (needs patch 0091); playback not listened to on 7.4 yet |
| ⚠️ | Speaker protection | V/I feedback mixer on, mode Normal, calibration loaded; not checked under load on 7.4 (see [Speakers](#speakers)) |
| ❔ | Cameras (OV02C10 RGB, HM1092 IR) | nodes are there, streaming not tested on 7.4 |
| ❔ | Suspend (s2idle) | not tested on 7.4. Lid wake quirk is in the EC driver |
| ❔ | USB-C data, charging, DP alt mode | not tested on 7.4 |
| ❌ | USB4 / Thunderbolt | disabled in the base (`ps883x`: "USB4 disabled until platform USB4 support is complete") |

✅ works ⚠️ partly or not fully checked ❔ not tested ❌ does not work

## Things that are broken in the 7.4-rc1 base (fixed here)

`qcom-laptops` is an integration branch, and `rc1` is its roughest point. On
this laptop the base breaks three things:

1. **Boot oops in Iris** (`NULL pointer dereference at 0x8` in
   `iris_vpu_boot_firmware`, when udev opens the decoder). 7.4 added
   `core->iris_firmware_data`, which is only set on the TrustZone (PAS) load
   path. The EL2 load path (patch 0016) now runs the same firmware detection.
2. **No sound card.** In 7.4 the sm8550 `rx_macro`/`tx_macro` require an `npl`
   clock that no DT has, and x1e80100 falls back to sm8550. Fixed by patch 0091.
3. **No Bluetooth.** `hamoa.dtsi` now has an empty `uart14_ep` endpoint, and
   `hci_qca` took any OF graph for an M.2 card, so it waited forever for an M.2
   power sequencer. Fixed by patch 0092.

## Building

```sh
git clone -b vivobook-s15-7.4 https://github.com/justririll/s15-x1e-linux
cd s15-x1e-linux
vivobook-s15/build.sh                                    # on the laptop
CROSS_COMPILE=aarch64-linux-gnu- vivobook-s15/build.sh   # or cross, on x86-64
```

The output goes to `build-vivobook/out/`: `vmlinuz-<ver>`, the DTB with the
overlays applied, and a modules tree with the external modules in `extra/`.

## Booting

- **ESP:** put the [slbounce](https://github.com/TravMurav/slbounce) EFI driver
  in `EFI/systemd/drivers/` (`vivobook-s15/boot/fetch-slbounce.sh` fetches and
  checks it). Put `tcblaunch.exe` from Windows at the ESP root.
- **Boot entry:** the systemd-boot entry must point at
  `x1e80100-asus-vivobook-s15-el2.dtb`, because slbounce only switches to EL2
  for an `-el2` DTB. Example: `vivobook-s15/boot/loader-entry.conf.example`.
- **Kernel command line:** in `vivobook-s15/boot/cmdline.txt`. `efi=noruntime`
  is required at EL2, so there are no EFI variables and `bootctl set-default`
  does not work.
- **Firmware:** not included, see `vivobook-s15/firmware/README.md`. You need
  the ASUS-signed ADSP/CDSP/GPU blobs and the speaker topology.

## Speakers

Speaker protection needs all of these:

- the topology from [x1e-speakers](https://github.com/justririll/x1e-speakers),
  installed as `qcom/x1e80100/X1E80100-ASUS-Vivobook-S15-tplg.bin`. The stock
  linux-firmware file with the same name has **no** protection;
- the `snd_soc_x1e80100` speaker-protection parameters, including the R0/T0
  calibration of your own unit;
- the DT overlay `40-wsa-sp-vi.dtso` (the V/I feedback DAI link).

Check before you turn the volume up:

```sh
dmesg | grep "speaker calibration"   # "speaker calibration (R0/T0) loaded"
amixer -c0 cget name='WSA WSA_AIF_VI Mixer WSA_SPKR_VI_1'        # values=on
amixer -c0 cget name='Speaker Protection Operation Mode'         # values=0 (Normal)
```

If the V/I feedback path is not running, mute the speakers. A small watchdog
that does this automatically (`spkr-prot-watch`) lives in the userspace part of
the setup, outside this kernel tree.

## Patches

`git log --reverse 98172d02f..vivobook-s15-7.4` has the whole series, each patch
with its rationale. In short:

| Range | What |
|---|---|
| 0001–0011 | EC/HID drivers, board DT (from the jglathe tree), Oryon DC ZVA erratum, EL2 SHM bridge, PCIe/pmdomain tweaks |
| 0012–0035 | ADSP/CDSP started by Linux at EL2: resets, halt regs, Q6 core clock controller, clk `sync_state`, Iris without TZ |
| 0036–0049 | DRM/msm DP/eDP fixes (HDR, BT.2020, SDP), Adreno devfreq and GMU tuning, schedutil |
| 0050–0056 | Cluster CL5 idle, Q6 PLLs from the active-only XO, timers, CDSP SPMI access |
| 0057–0084, 0089 | Speaker protection (WSA884x V/I sense, AudioReach SP modules, calibration, thermal zones) |
| 0085–0086 | RGB and IR cameras |
| 0087–0088 | Charge limit across ADSP restarts; EC per-profile CPU/GPU limits and lid wake quirk |
| 0090–0092 | Fixes for the 7.4-rc1 base (broken T14s DTB, LPASS `npl`, Bluetooth M.2 detection) |

The upstream `README` of the kernel is still in [`README`](README).
