#[[
add_library(kawalanlib)

if(MSVC)
  target_compile_options(kawalanlib PRIVATE /W4 /WX)
else()
  target_compile_options(kawalanlib PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()
]]