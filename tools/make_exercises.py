#!/usr/bin/env python3
"""Generate the reader's notebooks (2_notebooks/exercises) from the solved ones (2_notebooks/solutions).

In a solution notebook the parts the reader writes are fenced:

* code cells:      ``### BEGIN SOLUTION`` ... ``### END SOLUTION``
* markdown cells:  ``<!-- BEGIN SOLUTION -->`` ... ``<!-- END SOLUTION -->``

The fenced lines are replaced by a placeholder, all outputs are removed, and the result is written with the same
file name under exercises/. ``--check`` fails (exit code 1) when the committed exercises are out of date.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import nbformat

ROOT = Path(__file__).resolve().parents[1]
SOLUTIONS = ROOT / "2_notebooks" / "solutions"
EXERCISES = ROOT / "2_notebooks" / "exercises"

CODE_FENCE = re.compile(r"^(?P<indent>[ \t]*)### BEGIN SOLUTION[^\n]*\n.*?^[ \t]*### END SOLUTION[^\n]*(\n|$)",
                        re.MULTILINE | re.DOTALL)
MD_FENCE = re.compile(r"<!-- BEGIN SOLUTION -->.*?<!-- END SOLUTION -->", re.DOTALL)


def strip_code(source: str) -> str:
    def placeholder(match: re.Match) -> str:
        indent = match.group("indent")
        return f'{indent}# ✏️ your code here\n{indent}raise NotImplementedError("exercise")\n'

    return CODE_FENCE.sub(placeholder, source).rstrip("\n")


def strip_markdown(source: str) -> str:
    return MD_FENCE.sub("✏️ *Your answer here.*", source)


def make_exercise(solution: nbformat.NotebookNode) -> nbformat.NotebookNode:
    nb = nbformat.v4.new_notebook()
    nb.metadata = {
        "kernelspec": {"display_name": "Python 3", "language": "python", "name": "python3"},
        "language_info": {"name": "python"},
    }
    for cell in solution.cells:
        if cell.cell_type == "code":
            nb.cells.append(nbformat.v4.new_code_cell(strip_code(cell.source)))
        elif cell.cell_type == "markdown":
            nb.cells.append(nbformat.v4.new_markdown_cell(strip_markdown(cell.source)))
        else:
            nb.cells.append(nbformat.v4.new_raw_cell(cell.source))
    for k, cell in enumerate(nb.cells):
        cell.id = f"cell-{k:02d}"  # stable ids keep the files diff-friendly
    return nb


def render(nb: nbformat.NotebookNode) -> str:
    return nbformat.writes(nb) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="only check that exercises/ is up to date")
    args = parser.parse_args()

    EXERCISES.mkdir(parents=True, exist_ok=True)
    stale = []
    for path in sorted(SOLUTIONS.glob("*.ipynb")):
        text = render(make_exercise(nbformat.read(path, as_version=4)))
        target = EXERCISES / path.name
        if args.check:
            if not target.exists() or target.read_text(encoding="utf-8") != text:
                stale.append(target.relative_to(ROOT))
        else:
            target.write_text(text, encoding="utf-8")
            print(f"wrote {target.relative_to(ROOT)}")
    if stale:
        print("out of date (run `make exercises`):", *stale, sep="\n  ")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
