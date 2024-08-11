find_program(FORMATTER clang-format)
if(FORMATTER)
message(STATUS "Formatter found!")
  file(GLOB_RECURSE sourcefiles
    "include/*.hh"
    "src/*.cc"
    "grpc/*.cc"
    "grpc/*.hh")
  string (REPLACE ";" " " sourcefiles "${sourcefiles}")
  add_custom_target(code-format ALL
  COMMAND sh -c "clang-format -i ${sourcefiles}"
  VERBATIM)
endif()