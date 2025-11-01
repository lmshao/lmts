# Examples CMakeLists.txt for lmts

# TS Muxer Example
add_executable(ts_muxer_example ts_muxer_example.cpp)
target_link_libraries(ts_muxer_example PRIVATE lmts_static)
target_compile_features(ts_muxer_example PRIVATE cxx_std_17)

# TS Demuxer Example (with media file extraction)
add_executable(ts_demuxer_example ts_demuxer_example.cpp)
target_link_libraries(ts_demuxer_example PRIVATE lmts_static)
target_compile_features(ts_demuxer_example PRIVATE cxx_std_17)

# Install examples (optional)
if(LMTS_INSTALL_EXAMPLES)
    install(TARGETS ts_muxer_example ts_demuxer_example
        RUNTIME DESTINATION bin/examples
        COMPONENT examples
    )
endif()