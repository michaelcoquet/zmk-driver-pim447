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

    if(CONFIG_PIM447_STATUS_INDICATOR)
        # The status indicator subscribes to ZMK events (zmk/event_manager.h,
        # zmk/events/*). These headers live in the ZMK app's include directory,
        # which isn't on our module's include path by default - add it
        # explicitly. ZEPHYR_BASE points at the zephyr/ checkout, and ZMK is
        # a sibling of it in the West workspace layout used by ZMK configs.
        zephyr_library_include_directories(${ZEPHYR_BASE}/../zmk/app/include)
    endif()
endif()
