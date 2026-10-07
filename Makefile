PYTHON  ?= python3
BUILD   ?= build/cpp
JOBS    ?= $(shell nproc 2>/dev/null || echo 2)
CLANG_FORMAT ?= clang-format
FORMAT_STYLE := {BasedOnStyle: Google, IndentWidth: 4, ColumnLimit: 110, AccessModifierOffset: -2}
CPP_SOURCES  := $(shell find cpp python -name '*.cpp' -o -name '*.hpp' 2>/dev/null)

.PHONY: all build install test test-cpp test-python notebooks notebooks-inplace exercises figures format format-check clean

all: build test notebooks

## build: configure and compile the C++ library, tests and examples
build:
	cmake -S . -B $(BUILD) -DCMAKE_BUILD_TYPE=Release -DMC_BUILD_TESTS=ON -DMC_BUILD_EXAMPLES=ON
	cmake --build $(BUILD) -j $(JOBS)

## install: build and install the Python module in editable mode, with test and notebook extras
install:
	$(PYTHON) -m pip install -e ".[test,notebooks]"

## test: C++ tests (Catch2) and Python tests (bindings, Pinocchio cross-checks)
test: test-cpp test-python

test-cpp: build
	ctest --test-dir $(BUILD) --output-on-failure -j $(JOBS)

test-python:
	$(PYTHON) -m pytest -q

## notebooks: execute every solution notebook top to bottom and check the exercise copies are in sync
notebooks:
	$(PYTHON) tools/make_exercises.py --check
	$(PYTHON) tools/execute_notebooks.py 2_notebooks/solutions

## notebooks-inplace: execute the solution notebooks and store their outputs
notebooks-inplace:
	$(PYTHON) tools/execute_notebooks.py --inplace 2_notebooks/solutions

## exercises: regenerate 2_notebooks/exercises from the solution notebooks
exercises:
	$(PYTHON) tools/make_exercises.py

## figures: regenerate 1_theory/figures
figures:
	$(PYTHON) tools/make_figures.py

format:
	$(CLANG_FORMAT) -i --style="$(FORMAT_STYLE)" $(CPP_SOURCES)

format-check:
	$(CLANG_FORMAT) --dry-run --Werror --style="$(FORMAT_STYLE)" $(CPP_SOURCES)

clean:
	rm -rf build
