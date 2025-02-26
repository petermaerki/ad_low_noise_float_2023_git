# Available at setup time due to pyproject.toml
import setuptools
from pybind11.setup_helpers import Pybind11Extension, build_ext

__version__ = "0.1.7"

# The main interface is through Pybind11Extension.
# * You can add cxx_std=11/14/17, and then build_ext can be removed.
# * You can set include_pybind11=false to add the include directory yourself,
#   say from a submodule.
#
# Note:
#   Sort input source files if you glob sources to ensure bit-for-bit
#   reproducible builds (https://github.com/pybind/python_example/pull/53)

ext_modules = [
    Pybind11Extension(
        "ad_low_noise_float_2023_decoder",
        ["src/decoder.cpp"],
        # Example: passing in the version to the compiled code
        define_macros=[("VERSION_INFO", __version__)],
    ),
]

setuptools.setup(
    name="ad_low_noise_float_2023_decoder",
    version=__version__,
    author="Hans Maerki",
    author_email="buhtig.hans.maerki@ergoinfo.ch",
    url="https://github.com/petermaerki/ad_low_noise_float_2023_git/",
    description="ad_low_noise_float_2023_decoder",
    long_description="",
    ext_modules=ext_modules,
    # extras_require={"test": "pytest"},
    # Currently, build_ext only provides an optional "highest supported C++
    # level" feature, but in the future it may provide more features.
    cmdclass={"build_ext": build_ext},
    zip_safe=False,
    python_requires=">=3.7.9",
)
