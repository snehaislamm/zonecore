cmake_minimum_required(VERSION 3.16)
project(zonecore C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ZC_COVERAGE "Build with gcov instrumentation" OFF)

set(ZC_WARN -Wall -Wextra -Werror -pedantic -Wshadow -Wdouble-promotion -Wcast-qual
            -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wfloat-equal -Wswitch-enum)

file(GLOB_RECURSE ZC_SRC CONFIGURE_DEPENDS src/*.c)

# --- target-agnostic static library (what would be cross-compiled for the MCU)
add_library(zonecore STATIC ${ZC_SRC})
target_include_directories(zonecore PUBLIC include)
target_compile_options(zonecore PRIVATE ${ZC_WARN})
target_link_libraries(zonecore PUBLIC m)

# --- host shared library for SIL (adds host clock + fault-injection hooks)
add_library(zonecore_sil SHARED ${ZC_SRC})
target_include_directories(zonecore_sil PUBLIC include)
target_compile_options(zonecore_sil PRIVATE ${ZC_WARN})
target_compile_definitions(zonecore_sil PRIVATE ZC_HOST ZC_TEST_HOOKS)
target_link_libraries(zonecore_sil PUBLIC m)

# --- unit tests
file(GLOB ZC_UNIT CONFIGURE_DEPENDS tests/unit/*.c)
add_executable(zc_unit_tests ${ZC_UNIT} ${ZC_SRC})
target_include_directories(zc_unit_tests PRIVATE include tests/unit)
target_compile_options(zc_unit_tests PRIVATE ${ZC_WARN})
target_compile_definitions(zc_unit_tests PRIVATE ZC_TEST_HOOKS)
target_link_libraries(zc_unit_tests PRIVATE m)
if(ZC_COVERAGE)
  target_compile_options(zc_unit_tests PRIVATE --coverage -O0)
  target_link_options(zc_unit_tests PRIVATE --coverage)
endif()

enable_testing()
add_test(NAME unit COMMAND zc_unit_tests)
