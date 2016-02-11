# ============================================================================
# Copyright (c) 2009-2016 BlueQuartz Software, LLC
# ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */

set(${PLUGIN_NAME}Test_SOURCE_DIR ${${PLUGIN_NAME}_SOURCE_DIR}/Test)
set(${PLUGIN_NAME}Test_BINARY_DIR ${${PLUGIN_NAME}_BINARY_DIR}/Test)


#------------------------------------------------------------------------------
# Include this file from the SIMPL Project
include(${SIMPLProj_SOURCE_DIR}/Test/SIMPLTestFunctions.cmake)

include_directories(${${PLUGIN_NAME}_SOURCE_DIR}/Test)
include_directories(${${PLUGIN_NAME}_BINARY_DIR}/Test)
include_directories(${EbsdLibTest_BINARY_DIR})

set(TEST_TEMP_DIR ${${PLUGIN_NAME}_BINARY_DIR}/Test/Temp)
# Make sure the directory is created during CMake time
file(MAKE_DIRECTORY ${TEST_TEMP_DIR})

configure_file(${${PLUGIN_NAME}_SOURCE_DIR}/Test/TestFileLocations.h.in
               ${${PLUGIN_NAME}_BINARY_DIR}/Test/${PLUGIN_NAME}TestFileLocations.h @ONLY IMMEDIATE)

#------------------------------------------------------------------------------
# List all the source files here. They will NOT be compiled but instead
# be directly included in the main test source file. We list them here so that
# they will show up in IDEs
set(TEST_NAMES

)


set( ${PLUGIN_NAME}_TEST_SRCS )
set(FilterTestIncludes "")
set(TestMainFunctors "")

foreach(name  ${TEST_NAMES})
  set( ${PLUGIN_NAME}_TEST_SRCS
    ${${PLUGIN_NAME}_TEST_SRCS}
    "${${PLUGIN_NAME}_SOURCE_DIR}/Test/${name}.cpp"
    )
  string(CONCAT
    FilterTestIncludes
    ${FilterTestIncludes}
    "#include \"${name}.cpp\"\n"
    )

  string(CONCAT
    TestMainFunctors
   ${TestMainFunctors}
   "  ${name}()()|\n")
endforeach()

if(NOT "${TestMainFunctors}" STREQUAL "")
  STRING(REPLACE "|" ";" TestMainFunctors ${TestMainFunctors}   )
endif()

configure_file(${SIMPLProj_SOURCE_DIR}/Test/TestMain.cpp.in
               ${${PLUGIN_NAME}Test_BINARY_DIR}/${PLUGIN_NAME}UnitTest.cpp @ONLY)


# Set the source files properties on each source file.
foreach(f ${${PLUGIN_NAME}_TEST_SRCS})
  set_source_files_properties( ${f} PROPERTIES HEADER_FILE_ONLY TRUE)
endforeach()

AddSIMPLUnitTest(TESTNAME ${PLUGIN_NAME}UnitTest
  SOURCES ${${PLUGIN_NAME}Test_BINARY_DIR}/${PLUGIN_NAME}UnitTest.cpp ${${PLUGIN_NAME}_TEST_SRCS}
  FOLDER "${PLUGIN_NAME}Plugin/Test"
  LINK_LIBRARIES Qt5::Core Qt5::Gui H5Support SIMPLib)

if(MSVC)
  set_source_files_properties(${${PLUGIN_NAME}Test_BINARY_DIR}/${PLUGIN_NAME}UnitTest.cpp PROPERTIES COMPILE_FLAGS /bigobj)
endif()

