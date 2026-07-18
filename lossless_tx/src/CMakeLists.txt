FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.*)

idf_component_register(SRCS ${app_sources}
                       PRIV_REQUIRES espressif__usb_device_uac)

idf_component_get_property(uac_dir espressif__usb_device_uac COMPONENT_DIR)
idf_build_get_property(build_components BUILD_COMPONENTS)
if(tinyusb IN_LIST build_components)
    set(tinyusb_name tinyusb)
else()
    set(tinyusb_name espressif__tinyusb)
endif()
idf_component_get_property(tusb_lib ${tinyusb_name} COMPONENT_LIB)
target_include_directories(${tusb_lib} PUBLIC "${uac_dir}/tusb" "${uac_dir}/tusb_uac")
