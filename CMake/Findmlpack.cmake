# - Try to find ALSA
# Once done, this will define
#
#  ALSA_FOUND - system has ALSA (GL and GLU)
#  ALSA_INCLUDE_DIRS - the ALSA include directories
#  ALSA_LIBRARIES - link these to use ALSA
#  ALSA_GL_LIBRARY - only GL
#  ALSA_GLU_LIBRARY - only GLU
#
# See documentation on how to write CMake scripts at
# http://www.cmake.org/Wiki/CMake:How_To_Find_Libraries

include(LibFindMacros)
libfind_pkg_check_modules(MLPACK_PKGCONF mlpack)
find_path(MLPACK_INCLUDE_DIR
  NAMES mlpack/core.hpp
  PATHS ${MLPACK_PKGCONF_INCLUDE_DIRS}
)
find_library(MLPACK_LIBRARY
  NAMES mlpack
  PATHS ${MLPACK_PKGCONF_LIBRARY_DIRS}
)
# Extract the version number
set(MLPACK_PROCESS_INCLUDES MLPACK_INCLUDE_DIR)
set(MLPACK_PROCESS_LIBS MLPACK_LIBRARY)
libfind_process(MLPACK)
