
add_definitions(-std=c++11) # Use C++11
include_directories(include/smart_node)
include_directories(src)
include_directories(sdk/src/Sockets)
include_directories(sdk/src/impl/unix)
include_directories(sdk/include)




IF (WIN32)
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.h"
  "src/impl/windows/*.cpp"
  "src/impl/windows/*.h"
  "src/Sockets/*.cpp"
  "src/Sockets/*.h"
  "include/*.h"

)
	
ELSE()
FILE(GLOB SDK_SRC 
  "src/*.cpp"
  "src/*.h"
  "src/impl/unix/*.cpp"
  "src/impl/unix/*.h"
  "src/Sockets/*.cpp"
  "src/Sockets/*.h"
  "include/*.h"

)

	
ENDIF()

#add_subdirectory(samples)

#add_library(smart_driver SHARED ${SDK_SRC})
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
IF (WIN32)
ELSE()
target_link_libraries(smart_driver rt pthread)
ENDIF()                    
