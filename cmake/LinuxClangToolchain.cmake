set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_C_COMPILER   clang)
set(CMAKE_CXX_COMPILER clang++)

find_program(CMAKE_AR      NAMES llvm-ar      REQUIRED)
find_program(CMAKE_RANLIB  NAMES llvm-ranlib  REQUIRED)

set(CMAKE_EXE_LINKER_FLAGS    "-fuse-ld=lld" CACHE STRING "")
set(CMAKE_SHARED_LINKER_FLAGS "-fuse-ld=lld" CACHE STRING "")
set(CMAKE_MODULE_LINKER_FLAGS "-fuse-ld=lld" CACHE STRING "")
