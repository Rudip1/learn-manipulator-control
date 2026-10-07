#!/usr/bin/env python3
"""Execute notebooks top to bottom and fail on the first error.

    python tools/execute_notebooks.py 2_notebooks/solutions            # check only
    python tools/execute_notebooks.py --inplace 2_notebooks/solutions  # also store the outputs

Arguments are notebooks or directories of notebooks. Each notebook runs with its own directory as the working
directory, the way Jupyter runs it.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import nbformat
from nbclient import NotebookClient
from nbclient.exceptions import CellExecutionError


def collect(paths: list[str]) -> list[Path]:
    notebooks = []
    for p in map(Path, paths):
        notebooks += sorted(p.glob("*.ipynb")) if p.is_dir() else [p]
    return notebooks


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("paths", nargs="+")
    parser.add_argument("--inplace", action="store_true", help="write the executed notebook back")
    parser.add_argument("--timeout", type=int, default=900, help="per-cell timeout in seconds")
    args = parser.parse_args()

    failed = []
    for path in collect(args.paths):
        nb = nbformat.read(path, as_version=4)
        client = NotebookClient(nb, timeout=args.timeout, kernel_name="python3",
                                resources={"metadata": {"path": str(path.parent)}})
        start = time.perf_counter()
        try:
            client.execute()
        except CellExecutionError as err:
            print(f"FAIL {path}\n{err}")
            failed.append(path)
            continue
        print(f"ok   {path}  ({time.perf_counter() - start:.1f} s)")
        if args.inplace:
            nbformat.write(nb, path)
    if failed:
        print(f"{len(failed)} notebook(s) failed")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
