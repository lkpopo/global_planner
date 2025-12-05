# Get the directory of the current file. This is a robust way to get the path.
get_filename_component(MINIZ_ROOT_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

message(STATUS "miniz.c path is: ${MINIZ_ROOT_DIR}/miniz.c")

add_library(miniz_lib STATIC
    ${MINIZ_ROOT_DIR}/miniz.c
)

# PUBLIC properties are applied to the library itself and propagated to consumers.
# This ensures that any target linking against miniz_lib gets the correct include path
# and compile definitions.
target_include_directories(miniz_lib PUBLIC ${MINIZ_ROOT_DIR})
target_compile_definitions(miniz_lib PUBLIC MINIZ_HEADER_FILE_ONLY)