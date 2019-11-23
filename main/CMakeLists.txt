set(COMPONENT_SRCDIRS ".")
set(COMPONENT_ADD_INCLUDEDIRS ".")

register_component()

execute_process(COMMAND "date" "+%Y%m%d-%H%M%S"
    RESULT_VARIABLE date_result
    OUTPUT_VARIABLE date_output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

# GIT_COMMIT := $(shell git describe --match=NeVeRmAtCh --always --abbrev=14 --dirty)
execute_process(COMMAND "git" "describe" "--match=NeVeRmAtCh" "--always" "--abbrev=14" "--dirty"
    RESULT_VARIABLE git_result
    OUTPUT_VARIABLE git_output
    OUTPUT_STRIP_TRAILING_WHITESPACE)

message(STATUS "BUILD_TIMESTAMP [${date_result}]: ${date_output}")
message(STATUS "GIT_COMMIT [${git_result}]: ${git_output}")

target_compile_definitions(${COMPONENT_TARGET} PUBLIC BUILD_TIMESTAMP="${date_output}")
target_compile_definitions(${COMPONENT_TARGET} PUBLIC GIT_COMMIT="${git_output}")
