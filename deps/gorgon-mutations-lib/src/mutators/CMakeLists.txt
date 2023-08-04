
set(HEADER_LIST "${gorgon_SOURCE_DIR}/include/gorgon/mutators/base.h"
                "${gorgon_SOURCE_DIR}/include/gorgon/mutators/floats.h"
                "${gorgon_SOURCE_DIR}/include/gorgon/mutators/char_arrays.h"
                "${gorgon_SOURCE_DIR}/include/gorgon/mutators/numerical_arrays.h"
                "${gorgon_SOURCE_DIR}/include/gorgon/mutators/position_2d.h"
)

set(SRC_LIST    "floats.cpp"
                "char_arrays.cpp"
                "position_2d.cpp"
                "numerical_arrays.cpp"
)


add_library(gorgonMutators OBJECT ${SRC_LIST} ${HEADER_LIST})
set_property(TARGET gorgonMutators PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(gorgonMutators PUBLIC "${gorgon_SOURCE_DIR}/include/gorgon/mutators")
