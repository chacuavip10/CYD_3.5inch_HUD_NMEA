file(GLOB_RECURSE MAIN_SRCS *.c *.cpp)
file(GLOB_RECURSE UI_SRCS ui/**/*.c ui/**/*.cpp)  # Changed to include subdirectories

set(srcs ${MAIN_SRCS} ${UI_SRCS})

set(include_dirs
    .
    ui
    )

idf_component_register( SRCS ${srcs}
                      INCLUDE_DIRS ${include_dirs}
                      )
