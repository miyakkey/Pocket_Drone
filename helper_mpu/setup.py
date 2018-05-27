from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext
from Cython.Build import cythonize

setup(
    ext_modules = cythonize(
        Extension("cpphelper_calc",
                  sources=["cpphelper_calc.pyx", "cpphelper_calc_.cpp"],
                  language="c++",
                 )
    )
)