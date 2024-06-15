# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_C_STANDARD_REQUIRED ON)
# set(CMAKE_C_EXTENSIONS ON)

include_directories(./)



file (GLOB XSERIAL_SRC 
    "./*.cpp"
    "./*.hpp"
    "./*.h" 
)

#set(XSERIAL_LIB ${PROJECT_NAME}-xserial)
# add_library(XSERIAL_SHLIB SHARED ${XSERIAL_SRC}) 
add_library(XSERIAL_STLIB STATIC ${XSERIAL_SRC} )

message("********" ${XSERIAL_SRC})

