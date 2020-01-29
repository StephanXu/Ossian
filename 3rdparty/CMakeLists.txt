# import HKCamera library
add_library(HKCamera UNKNOWN IMPORTED)

find_library(HKCamera_LIB MvCameraControl)
find_path(HKCamera_INCLUDE MvCameraControl.h)
message("FOUND HKCamera: ${HKCamera_LIB}")
message("FOUND HKCamera headers: ${HKCamera_INCLUDE}")

set_property(TARGET HKCamera PROPERTY IMPORTED_LOCATION "${HKCamera_LIB}")
set_property(TARGET HKCamera PROPERTY INTERFACE_INCLUDE_DIRECTORIES "${HKCamera_INCLUDE}")

target_link_libraries(Ossian
        PUBLIC
            HKCamera
        )