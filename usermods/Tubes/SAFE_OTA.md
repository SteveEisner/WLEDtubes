# Safe Tubes OTA

`safe_ota.py` is the only supported OTA path for Tubes fleet firmware.
It is intentionally conservative: if it cannot prove the board, config, and
firmware are compatible, it refuses to upload.

The safe path is:

1. Fetch `/json/si` and `/json/cfg`.
2. Identify the hardware from release/config markers.
3. Back up the current info and config to `/tmp/WLED-Tubes-ota-backups/`.
4. Install a known-safe hardware/boot config with `/json/cfg`. This config
   uses the correct hardware pins but boots dark (`def.on=false`, brightness 5)
   so the new firmware can prove it starts before any LED load is applied.
5. Reboot once and verify the safe config survives.
6. For pre-16 devices, stage through regular WLED 16 first.
7. Upload the Tubes firmware only after the device is on WLED 16 and the safe
   config is still present.
8. Verify the final release, `vid`, LED count, config, and short-term uptime.

The old `firmware.sh upload` remote autoupdate path is disabled because it cannot
preflight or migrate `/cfg.json` on each pole before flashing.

Examples:

```sh
./safe_ota.py preflight 4.3.2.1
./safe_ota.py update 4.3.2.1
./firmware.sh batch
```
