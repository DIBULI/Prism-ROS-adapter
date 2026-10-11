# Keep legal notices alongside each installed ROS package.
get_filename_component(_prism_license_root
  "${CMAKE_CURRENT_LIST_DIR}/../.." ABSOLUTE)
install(FILES
  "${_prism_license_root}/LICENSE"
  "${_prism_license_root}/NOTICE"
  "${_prism_license_root}/LICENSING.md"
  "${_prism_license_root}/THIRD_PARTY_NOTICES.md"
  DESTINATION "share/${PROJECT_NAME}")
install(DIRECTORY "${_prism_license_root}/licenses/"
  DESTINATION "share/${PROJECT_NAME}/licenses")
unset(_prism_license_root)
