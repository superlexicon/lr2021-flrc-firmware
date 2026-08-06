#!/usr/bin/env bash
# Helper script to flash the pre-built FLRC hardware burst firmware binary (bin/zephyr.hex)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEX_PATH="${SCRIPT_DIR}/bin/zephyr.hex"

if [ ! -f "${HEX_PATH}" ]; then
    echo "Error: Firmware binary not found at ${HEX_PATH}"
    exit 1
fi

PYOCD_CMD="pyocd"
if [ -f "/home/mano/workspace/lr2021-firmware/.venv/bin/pyocd" ]; then
    PYOCD_CMD="/home/mano/workspace/lr2021-firmware/.venv/bin/pyocd"
fi

if [ -n "$1" ]; then
    PROBES=("$1")
else
    # Default probe IDs if no argument passed
    PROBES=("7C1E0AEB" "071BAB41")
fi

echo "========================================================================="
echo " Flashing Pre-built FLRC Firmware: ${HEX_PATH}"
echo "========================================================================="

for probe in "${PROBES[@]}"; do
    echo "--> Flashing target probe: ${probe}..."
    ${PYOCD_CMD} flash -t nrf54l -u "${probe}" "${HEX_PATH}"
    ${PYOCD_CMD} reset -t nrf54l -u "${probe}"
done

echo "========================================================================="
echo " FLASHING COMPLETE! All target boards reset and ready."
echo "========================================================================="
