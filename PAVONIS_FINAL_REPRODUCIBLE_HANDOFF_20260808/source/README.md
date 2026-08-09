# Source Package

`bundles/` contains history-free Git snapshots for the four independently
versioned repositories. `patches/` contains the Pavonis changes as reviewable
patch series from the recorded clean proof source revisions.
`overlays/ocudu-cp3406/` contains the exact final Soapy TX source overlay used
for the CP3410-CP3424 hardware-throughput closure.

Use the bundles for a reproducible checkout from the release root:

```bash
./tools/unpack_source.sh "$HOME/pavonis-source"
```

The extractor verifies these submission heads before applying the separate
CP3406 OCUDU overlay:

| Repository | Commit |
|---|---|
| LiteX M2SDR | `b4d3dbae113ed8a6a33fac177dab0d85381677d6` |
| OCUDU | `d12a7f10ec4edfb57364e7be31343da34fafb81d` |
| qcore | `d84e2f433c8ff9f5fb10e6fcc05d25b6c15a7c67` |
| srsRAN 4G | `d3a772af794650cd2544a88ee0781f7af0bb219a` |

The package records three revision classes:

- **proof source**: source revision used by the established campaign
  deployment before the cleanup commits;
- **hardening tip**: clean local review branch containing the minimal retained
  Pavonis changes and default-off diagnostics;
- **submission snapshot**: deterministic root commit used by the bundle, with
  no reachable repository history.

The OCUDU, srsRAN, and LiteX snapshot trees are byte-identical to their
hardening tips. The qcore snapshot omits two tracked `sims.toml` fixtures and
one documentation packet capture to satisfy the submission privacy rule; none
is required to build qcore. The original patch series remains the review
record.

The hardening tips built successfully at CP3119, but they were not silently
substituted for the campaign deployment. Treat migration from the proof
deployment to the submission snapshot as a staged revalidation, beginning
with ZMQ. Exact revisions are verified by `tools/unpack_source.sh` and written
to the extracted workspace's `SOURCE_REVISIONS` file.

For the final MCS19 hardware profile, apply the CP3406 overlay after unpacking
and before the native OCUDU build. Its README records exact source hashes and
the focused default-off deadline test. The original bundles and patch series
remain unchanged for provenance.

Build and staged-validation commands are in `../docs/RUNBOOK.md`. The
repositories retain their own licenses; see `../docs/LICENSES.md`.
