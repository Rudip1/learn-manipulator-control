# How to run

## Requirements

- A C++17 compiler, CMake ≥ 3.20
- Eigen 3.4 (`sudo apt install libeigen3-dev`; if it is missing, CMake downloads the headers)
- Python ≥ 3.10 with `pip`

Catch2 v3 (for the C++ tests) is used from the system when installed (`sudo apt install catch2`) and fetched
otherwise. [Pinocchio](https://github.com/stack-of-tasks/pinocchio) (`pip install pin`) is only a reference in
tests and notebooks; the library does not depend on it.

## Python module

```bash
git clone https://github.com/Rudip1/learn-manipulator-control
cd learn-manipulator-control
pip install -e ".[test,notebooks]"     # builds the C++ core with scikit-build-core + pybind11
python -c "import manipulator_control as mc; print(mc.rot_z(0.5))"
```

The editable install does not rebuild by itself: after changing C++ code run the `pip install -e .` line again.

## C++ library, tests and examples

```bash
make build        # cmake -S . -B build/cpp ... && cmake --build build/cpp
make test         # Catch2 tests (ctest) + Python tests (pytest, including the Pinocchio cross-checks)
./build/cpp/cpp/examples/01_transforms
```

To use the library from another CMake project, add this repository with `add_subdirectory` or `FetchContent`
and link `manipulator_control::manipulator_control`.

## Notebooks

```bash
jupyter lab 2_notebooks/exercises      # the reader's copies, with ✏️ cells to fill in
make notebooks                          # execute every solution notebook top to bottom (what CI runs)
```

On Colab, open a notebook from `2_notebooks/exercises/` (each one has an "Open in Colab" badge). Its first cell
installs the module from GitHub, which compiles the C++ core; this takes a few minutes the first time.

## Maintenance

| Command | What it does |
|---|---|
| `make exercises` | regenerate `2_notebooks/exercises/` from the solution notebooks (fenced `BEGIN/END SOLUTION` blocks become ✏️ placeholders, outputs are removed) |
| `make notebooks-inplace` | execute the solution notebooks and store their outputs |
| `make figures` | regenerate `1_theory/figures/` |
| `make format` / `make format-check` | clang-format (Google base, 4-space indent, 110 columns) |
