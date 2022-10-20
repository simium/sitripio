add_executable(sitripio
        main.c
        )

# pull in common dependencies and additional uart hardware support
target_link_libraries(sitripio pico_stdlib hardware_uart)

# enable usb output, enable uart output
pico_enable_stdio_usb(sitripio 1)
pico_enable_stdio_uart(sitripio 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(sitripio)

# add url via pico_set_program_url
example_auto_set_url(sitripio)
