add_executable(sitripio
        main.c
        )
        
add_subdirectory("/Users/jotape/pico/pico-lorawan" "/Users/jotape/pico/pico-lorawan/build" EXCLUDE_FROM_ALL)

# pull in common dependencies and additional uart hardware support
target_link_libraries(sitripio pico_stdlib hardware_adc hardware_uart hardware_gpio pico_lorawan)

# enable usb output, enable uart output
pico_enable_stdio_usb(sitripio 1)
pico_enable_stdio_uart(sitripio 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(sitripio)

# add url via pico_set_program_url
example_auto_set_url(sitripio)
