add_library( simple_vo SHARED
    frame.cpp
    mappoint.cpp
    map.cpp
    camera.cpp
    config.cpp
    g2o_types.cpp
    vo.cpp
)

target_link_libraries( simple_vo
    ${THIRD_PARTY_LIBS}
)