set (excluded_examples
)

gtsamAddExamplesGlob("*.cpp" "${excluded_examples}" "gtsam;${Boost_PROGRAM_OPTIONS_LIBRARY}")
