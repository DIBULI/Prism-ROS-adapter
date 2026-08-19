#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "Prism::UsbSdk" for configuration "Release"
set_property(TARGET Prism::UsbSdk APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(Prism::UsbSdk PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libprism_usb_sdk.so"
  IMPORTED_SONAME_RELEASE "libprism_usb_sdk.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS Prism::UsbSdk )
list(APPEND _IMPORT_CHECK_FILES_FOR_Prism::UsbSdk "${_IMPORT_PREFIX}/lib/libprism_usb_sdk.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
