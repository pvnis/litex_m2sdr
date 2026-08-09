# Historical Material

This directory is secondary reference material, not the operator entry point.

`sanitized-checkpoints-final/` is the authoritative atomic privacy-clean
corpus. It contains every available report through
CP3637: 3,587 reports spanning 3,586 checkpoint numbers, with two distinct
CP0072 reports. The source corpus has no CP0001-CP0016 report files, and
CP3633 has artifacts but no checkpoint report, so none are invented here.
Its single `HASH_MANIFEST.tsv` binds every original source report to the final
sanitized bytes and is documented by its local `SANITIZATION_NOTES.md`.

`sanitized-checkpoints/README.md` is only a redirect for historical links. The
superseded layered copies are not duplicated in this final package; their
original campaign artifacts and prior supervisor package remain untouched.

Earlier cold-lab guidance is represented by its sanitized checkpoint records.
It contained superseded forced-AL4 guidance and is intentionally not
republished as an operator document. Use `../docs/RUNBOOK.md`, which carries
the final CP3637 staged release procedure.
