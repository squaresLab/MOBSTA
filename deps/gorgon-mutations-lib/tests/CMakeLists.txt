enable_testing()

# use a local installation rather than fetching one
if(DEFINED LOCAL_GTEST_LIB)
    find_package(GTest 
            NAMES GTest
            HINTS "${LOCAL_GTEST_LIB}"
    )

    set(GTest_TARGET GTest::gtest_main)
else()
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG release-1.12.1
    )
    FetchContent_MakeAvailable(googletest)

    set(GTest_TARGET GTest::gtest_main)
endif()

set(TEST_LIST 
    "mutators_floats_test.cpp"
    "mutators_char_arrays_test.cpp"
    "mutators_position_2d_test.cpp"
    "mutators_numeric_test.cpp"
    "mutators_numeric_arrays_test.cpp"
)


add_executable( mutators_test ${TEST_LIST})

target_link_libraries(
    mutators_test
    PRIVATE gorgon
    ${GTest_TARGET}
    ${nlohmann_json_TARGET}
)


include(GoogleTest)
gtest_discover_tests(mutators_test
    EXTRA_ARGS "--gtest_repeat=64" "--rerun_failed" "--output-on-failure"
)
