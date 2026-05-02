# BioXP OEM Compatibility Dry-Run Container

This container is for workstation/robot dry-run verification only. It must not be used as a live hardware container.

Build:

```bash
docker build -f Dockerfile.oem-compat-dryrun -t bioxp-oem-compat:dryrun .
```

Run dry-run API, no USB device mounts:

```bash
docker run --rm -p 8080:8080 \
  -e BIOXP_MODE=dry_run \
  -e BIOXP_HARDWARE_ENABLE=0 \
  bioxp-oem-compat:dryrun
```

Smoke tests:

```bash
curl -s -X POST http://127.0.0.1:8080/oem-compat/startup/dry-run \
  -H 'content-type: application/json' \
  -d '{"run_homing": true}'
```

Safety rules:

- no `/dev/bus/usb` mount in this image/profile
- default mode is `dry_run`
- live/shadow hardware access requires a separate profile and explicit operator gate later
- BMS should treat this as a compatibility runtime/control-plane preview, not hardware truth
