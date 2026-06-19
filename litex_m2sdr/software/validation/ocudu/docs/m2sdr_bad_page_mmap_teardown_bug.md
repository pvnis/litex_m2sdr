# M2SDR bad-page mmap teardown bug

## Symptom

After Soapy/OCUDU validation exits, the kernel logs repeated warnings:

```text
BUG: Bad page map in process radio_ssb
file:m2sdr0 fault:0x0 mmap:litepcie_mmap [m2sdr]
```

The TX path itself can be clean:

```text
[TX] Overflow: 0; Late: 0 Underflow: 0 Other: 0
Timed TX counters: Late=0, Underruns=0, Dropped=0, State=IDLE
```

So this is tracked separately from RF waveform validity.

## Current containment

- Avoid LitePCIe copy-mode on nuc4; it hung the Soapy/M2SDR path.
- Keep `M2SDR_SOAPY_TX_TIME_OFFSET_NS=20000000` for `radio_ssb`.
- Keep kernel warnings visible in validation summaries.
- Do not remotely unload/reload the M2SDR kernel module to test a patch.

## Candidate root-cause area

The warning names `litepcie_mmap [m2sdr]`, so inspect the kernel driver's mmap path:

- `litepcie_mmap`
- `.mmap = ...`
- `remap_pfn_range` / `io_remap_pfn_range` / `dma_mmap_*`
- VMA flags such as `VM_IO`, `VM_PFNMAP`, `VM_DONTEXPAND`, `VM_DONTDUMP`
- teardown/free order of DMA buffers versus active user VMAs

## Candidate fix if source confirms missing VMA flags

If the driver maps device/DMA PFNs with `remap_pfn_range()` and does not mark the VMA as PFN/device I/O mapping, add the appropriate VMA flags before remapping:

```c
vma->vm_flags |= VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP;
vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
```

Use `pgprot_writecombine()` instead of `pgprot_noncached()` only if the existing driver already uses write-combining consistently for these buffers.

## Validation path for a real fix

1. Inspect the mmap source with `scripts/inspect_m2sdr_bad_page_mmap.sh`.
2. Patch source only after confirming the actual mapping path.
3. Build the kernel module.
4. Do not test by remote module unload/reload on nuc4.
5. Test only after a planned reboot or with physical/out-of-band recovery available.
6. Re-run:
   - Soapy loopback teardown test
   - `radio_ssb` short run
   - `journalctl -k` bad-page grep
## Source inspection finding: 2026-06-19

Inspection of `litex_m2sdr/software/kernel/main.c` found `litepcie_mmap()` at the DMA mmap path. On non-ARM/x86 it maps each DMA buffer by creating a temporary `struct vm_area_struct sub_vma = *vma` and calling `dma_mmap_coherent()` for each chunk. Kernel logs show the failing path as:

```text
BUG: Bad page map in process radio_ssb
file:m2sdr0 fault:0x0 mmap:litepcie_mmap [m2sdr]
```

The source did not mark the original VMA with `VM_IO`, `VM_PFNMAP`, `VM_DONTEXPAND`, or `VM_DONTDUMP` before installing DMA/PFN mappings. Because teardown uses the original VMA, not the temporary `sub_vma`, this is a contained candidate cause for the bad-page warnings.

Candidate source fix applied here:

```c
litepcie_set_dma_mmap_flags(vma);
```

where the helper marks the original VMA as:

```c
VM_IO | VM_PFNMAP | VM_DONTEXPAND | VM_DONTDUMP
```

This source patch is not a runtime fix until the kernel module is rebuilt, installed, and loaded by a planned reboot or another controlled recovery path. Do not test by remote `rmmod`/`modprobe -r`/`insmod` sequencing on nuc4.
