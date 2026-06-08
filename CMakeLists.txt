# zmk-driver-pim447/CMakeLists.txt
# Top-level build file for the PIM447 trackball driver module.

zephyr_include_directories(include)

if(CONFIG_PIM447)
    zephyr_library()
    zephyr_library_sources(drivers/input/pim447.c)
endif()
