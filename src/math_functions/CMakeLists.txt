add_library(math_functions my_sqrt.cc)
target_include_directories(math_functions
                           INTERFACE
                           "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
                           "$<INSTALL_INTERFACE:include>"
                           )
target_compile_definitions(math_functions
                           PRIVATE "ASTAR_BUILD"
                           )
target_link_libraries(math_functions PUBLIC "${PROJECT_NAME}_options")


install(TARGETS math_functions ${PROJECT_NAME}_options
        DESTINATION lib
        EXPORT math_functionsTargets)
install(FILES my_sqrt.hpp DESTINATION include)