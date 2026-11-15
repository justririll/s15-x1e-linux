# Firmware (not included)

The blobs are not redistributable, so only their checksums are here (`SHA256SUMS`).

| File | Where to get it | Why |
|---|---|---|
| `qcom/x1e80100/ASUSTeK/vivobook-s15/*.mbn, *.elf, *.jsn` | the ASUS Windows driver package for the S5507QA (or `C:\Windows\System32\DriverStore\FileRepository` on the preinstalled Windows) | vendor-signed ADSP, CDSP, GPU zap shader; linux-firmware has none for this model. Paths come from `firmware-name` in the DT |
| `qcom/x1e80100/X1E80100-ASUS-Vivobook-S15-tplg.bin` | built from [x1e-speakers](https://github.com/justririll/x1e-speakers) | ALSA topology **with speaker protection**. The stock linux-firmware topology of the same name has none |

Install into `/usr/lib/firmware/updates/` so it wins over the distribution's files.
