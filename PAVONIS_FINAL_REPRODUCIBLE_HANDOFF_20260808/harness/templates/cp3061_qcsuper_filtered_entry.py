#!@REMOTE_HOME@/pavonis_qcsuper_cp2741/bin/python3
"""Run QCSuper DLF capture with only the Pavonis NR access log codes enabled."""

from __future__ import annotations

import sys

from qcsuper.modules import dlf_dump


LOG_CODES = frozenset({0xB821, 0xB889, 0xB88A})
_original_init = dlf_dump.DlfDumper.__init__


def _filtered_init(self, diag_input, dlf_file):
    _original_init(self, diag_input, dlf_file)
    self.limit_registered_logs = LOG_CODES


dlf_dump.DlfDumper.__init__ = _filtered_init

from qcsuper.main import main  # noqa: E402


print("PAVONIS_QCSUPER_FILTERED_CODES=b821,b889,b88a", flush=True)
sys.exit(main())
