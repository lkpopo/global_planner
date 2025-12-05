# Get the directory of the current file and correct the variable name typo.
get_filename_component(PUGIXML_ROOT_DIR ${CMAKE_CURRENT_LIST_FILE} DIRECTORY)

# Add a library target for pugixml.
add_library(pugixml_lib STATIC
    ${PUGIXML_ROOT_DIR}/pugixml.cpp
)

# Propagate include directory to consumers of this library.
target_include_directories(pugixml_lib PUBLIC ${PUGIXML_ROOT_DIR})


