import pathlib
import sysconfig

# include directory
inc = f"-I{sysconfig.get_paths()['include']}"

# library directory  & library name
libdir = sysconfig.get_config_var("LIBDIR")
ldlibrary = sysconfig.get_config_var("LDLIBRARY")  # e.g. "libpython3.12.so"
libswitches = f"-L{libdir} -l{pathlib.Path(ldlibrary).stem.lstrip('lib')}"
# print(inc, libswitches)

cpp_args = [
    inc,
    f"-L{libdir}",
    f"-l{pathlib.Path(ldlibrary).stem.lstrip('lib')}",
]
print("\n".join(f'"{arg}",' for arg in cpp_args))
