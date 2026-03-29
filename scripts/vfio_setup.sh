#!/bin/sh

set -eu

PCI_ADDR="${PCI_ADDR:-0000:01:00.0}"
VENDOR_DEVICE="${VENDOR_DEVICE:-10ee 7021}"
DEVICE_SYSFS="/sys/bus/pci/devices/${PCI_ADDR}"
VFIO_SYSFS="/sys/bus/pci/drivers/vfio-pci"

if [ "$(id -u)" -ne 0 ]; then
    echo "run as root" >&2
    exit 1
fi

modprobe vfio-pci

if [ -L "${DEVICE_SYSFS}/driver" ]; then
    CURRENT_DRIVER="$(basename "$(readlink -f "${DEVICE_SYSFS}/driver")")"
    if [ "${CURRENT_DRIVER}" != "vfio-pci" ]; then
        printf '%s' "${PCI_ADDR}" > "${DEVICE_SYSFS}/driver/unbind"
    fi
fi

printf '%s' "${VENDOR_DEVICE}" > "${VFIO_SYSFS}/new_id" 2>/dev/null || true

if [ ! -L "${VFIO_SYSFS}/${PCI_ADDR}" ]; then
    printf '%s' "${PCI_ADDR}" > "${VFIO_SYSFS}/bind"
fi

lspci -nn -s "${PCI_ADDR#0000:}" -k
