# Cell Equivalence Checklist

Keep band, ARFCNs, SCS, PLMN, TAC, PCI, PRACH index, SIM profile, and qcore
provisioning consistent unless the experiment explicitly changes one variable.
Record every intentional difference in the run result.

| Parameter | ZMQ baseline | OTA Band 3 starting point |
| --- | --- | --- |
| Band | n78 | 3 |
| DL ARFCN | 626000 | 368500 |
| SSB ARFCN | 625632 | 367930 |
| Bandwidth | 20 MHz | 10 MHz |
| SCS | 30 kHz | 15 kHz |
| PLMN | 00101 | 00101 |
| TAC | 1 | 1 |
| PCI | 1 | 1 |
| PRACH config index | 159 | 1 |
| Sample rate | 23.04 Msps | 11.52 Msps |

For M2SDR/Soapy tests, preserve `M2SDR_DIRECT_DEINTERLEAVE=1`.
