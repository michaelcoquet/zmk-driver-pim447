# zmk-driver-pim447/CMakeLists.txt
# Top-level build file for the PIM447 trackball driver module.

# Make drivers/input/pim447.h discoverable by consumers via
#   #include <drivers/input/pim447.h>
zephyr_include_directories(include)

if(CONFIG_PIM447)
    zephyr_library()
    zephyr_library_sources(drivers/input/pim447.c)
    zephyr_library_sources_ifdef(CONFIG_PIM447_STATUS_INDICATOR
                                 drivers/input/pim447_status.c)
endif()
