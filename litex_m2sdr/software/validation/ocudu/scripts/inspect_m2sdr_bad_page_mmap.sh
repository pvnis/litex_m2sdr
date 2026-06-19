#!/usr/bin/env bash
set -euo pipefail

cd "$HOME/CLionProjects/m2sdr"

echo "===== kernel mmap source search ====="
find . -path '*/software/kernel/*' -type f \
  \( -name '*.c' -o -name '*.h' \) \
  -print | sort

echo
echo "===== mmap/remap/vma references ====="
grep -RInE 'litepcie_mmap|\.mmap|remap_pfn_range|io_remap_pfn_range|dma_mmap|vm_flags|VM_IO|VM_PFNMAP|VM_DONTEXPAND|VM_DONTDUMP|pgprot_' \
  litex_m2sdr/software/kernel 2>/dev/null || true

echo
echo "===== current m2sdr module/device state ====="
uname -a
ls -l /dev/m2sdr* 2>/dev/null || true
lsmod | grep -E '^m2sdr|litepcie' || true
modinfo m2sdr 2>/dev/null | sed -n '1,80p' || true

echo
echo "===== recent bad-page messages ====="
sudo journalctl -k --no-pager -n 3000 | \
  grep -iE 'bad page|litepcie_mmap|m2sdr0|BUG: Bad page|radio_ssb|m2sdr' | tail -200 || true
