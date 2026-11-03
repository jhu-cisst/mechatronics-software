project(Amp1394SimulationLib)

set(SOURCE_FILES
    code/SimulationPort.cpp)

include_directories(
    "."
    ${Amp1394_INCLUDE_DIR} 
    ${Amp1394_EXTRA_INCLUDE_DIR})

# Create Amp1394 simulation library
add_library(Amp1394Simulation ${SOURCE_FILES})
target_link_libraries(Amp1394Simulation ${Amp1394_EXTRA_LIBRARIES})