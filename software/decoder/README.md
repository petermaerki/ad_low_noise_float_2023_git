# Encoder

See:
* https://github.com/pybind/python_example
* https://cibuildwheel.readthedocs.io

The pic encodes the measurements sent over USB.

This decoder runs on the PC and decodes the measurements.

The decoder distributed in a python wheel.

## Version number

See: setup.py

## Local build and test

```bash
uv venv --python 3.13.3 .venv
. .venv/bin/activate
uv pip install -e .
python tests/test.py
```

## Local build and test

```bash
uv venv --python 3.13.3 .venv
. .venv/bin/activate
uv pip install cibuildwheel==2.23.3
./run_build_and_test.sh
```
