#!/usr/bin/env bash
# One time USB migration of an ESPSomfy-RTS controller (ESP32-S3, 8MB flash or more) from the
# stock 4MB partition layout to the Matter layout in partitions.csv.
#
# Settings (nvs at 0x9000) and shades (LittleFS at 0x290000) stay where they are.  Only the
# bootloader, partition table and firmware are written; otadata and the new Matter nvs
# partition are erased.  The first 4MB (everything the stock layout uses) is backed up first.
#
# usage: scripts/migrate-to-matter.sh <serial port> <dir with release binaries>
#   The directory must contain the release assets:
#     SomfyController.bootloader.esp32s3.bin
#     SomfyController.partitions.esp32s3.bin
#     SomfyController.ino.esp32s3.bin
#
# Restore the backup with:
#   python3 -m esptool --chip esp32s3 --port <port> write_flash 0x0 <backup file>
set -euo pipefail

PORT=${1:-}
DIR=${2:-}
if [[ -z "$PORT" || -z "$DIR" ]]; then
  sed -n '2,19p' "$0" | sed 's/^# \{0,1\}//'
  exit 1
fi
ESPTOOL=${ESPTOOL:-python3 -m esptool}
BOOTLOADER="$DIR/SomfyController.bootloader.esp32s3.bin"
PARTITIONS="$DIR/SomfyController.partitions.esp32s3.bin"
FIRMWARE="$DIR/SomfyController.ino.esp32s3.bin"
for f in "$BOOTLOADER" "$PARTITIONS" "$FIRMWARE"; do
  [[ -f "$f" ]] || { echo "Missing $f"; exit 1; }
done
esptool() { $ESPTOOL --chip esp32s3 --port "$PORT" "$@"; }

# The firmware must end before the LittleFS partition at 0x290000 or flashing would overwrite it.
fwsize=$(wc -c < "$FIRMWARE")
if (( fwsize > 0x280000 )); then
  echo "Firmware is $fwsize bytes, larger than the 0x280000 app partition"; exit 1
fi

echo "== Checking flash size"
flashsize=$(esptool flash_id | sed -n 's/^Detected flash size: \([0-9]*\)MB.*/\1/p')
if [[ -z "$flashsize" || "$flashsize" -lt 8 ]]; then
  echo "This layout needs at least 8MB of flash, detected '${flashsize}MB'"; exit 1
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

echo "== Checking the current partition table"
esptool read_flash 0x8000 0xC00 "$WORK/current-partitions.bin"
python3 -I - "$WORK/current-partitions.bin" <<'EOF'
import struct, sys
data = open(sys.argv[1], 'rb').read()
parts = {}
for i in range(0, len(data), 32):
    e = data[i:i + 32]
    if e[:2] != b'\xaa\x50':
        break
    offset, size = struct.unpack('<II', e[4:12])
    parts[e[12:28].rstrip(b'\0').decode()] = (e[2], e[3], offset, size)
# Already migrated: espsomfy (old nvs) at 0x9000.
if parts.get('espsomfy') == (0x01, 0x02, 0x9000, 0x5000):
    print('The Matter partition layout is already installed.  Update the firmware from the web ui instead.')
    sys.exit(2)
expected = {'nvs': (0x01, 0x02, 0x9000, 0x5000), 'spiffs': (0x01, 0x82, 0x290000, 0x160000)}
for name, want in expected.items():
    if parts.get(name) != want:
        print(f'Unexpected partition table, {name} is {parts.get(name)} not {want}.  Not migrating.')
        sys.exit(1)
print('Stock layout found: settings at 0x9000 and LittleFS at 0x290000 will be kept.')
EOF

BACKUP="espsomfy-backup-$(date +%Y%m%d-%H%M%S).bin"
# The stock layout ends at 0x400000, so this is enough to restore the previous firmware and data.
echo "== Backing up the stock layout (first 4MB) to $BACKUP"
esptool read_flash 0x0 0x400000 "$BACKUP"

echo "== Writing the Matter layout"
# Erased otadata boots app0 (the previous firmware may have been running from app1 at 0x150000).
# The new nvs partition may hold leftovers from older firmware on 8MB+ boards.
esptool erase_region 0xe000 0x2000
esptool erase_region 0x680000 0x20000
esptool write_flash --flash_mode keep --flash_size keep \
  0x0 "$BOOTLOADER" \
  0x8000 "$PARTITIONS" \
  0x10000 "$FIRMWARE"

echo "== Done.  Keep $BACKUP until you have verified the controller."
echo "   Then update the file system from the web ui (Update Application) to get the Matter settings page."
