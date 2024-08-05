find_program(FORMATTER clang-format)
if(FORMATTER)
message(STATUS "Formatter found!")
  file(GLOB_RECURSE sourcefiles
    "include/*.hh"
    "src/*.cc")
  string (REPLACE ";" " " sourcefiles "${sourcefiles}")
  add_custom_target(format ALL
  COMMAND sh -c "clang-format -i ${sourcefiles} --verbose"
  VERBATIM)
endif()