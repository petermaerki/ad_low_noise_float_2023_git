set -euox pipefail

. .venv/bin/activate
# Failes... Why?
python -m cibuildwheel --only cp313-manylinux_x86_64 --output-dir wheelhouse

uv pip install --upgrade wheelhouse/ad_low_noise_float_2023_decoder-0.1.1-cp313-cp313-manylinux_2_17_x86_64.manylinux_2_28_x86_64.whl
python tests/test.py

python -m cibuildwheel --only cp37-manylinux_x86_64 --output-dir wheelhouse

uv pip install --upgrade wheelhouse/ad_low_noise_float_2023_decoder-0.1.1-cp313-cp313-manylinux_2_17_x86_64.manylinux_2_28_x86_64.whl
python tests/test.py
