

set(augmSSL_SOURCES
    AugmSSL.cpp
    FieldLineDetector.cpp
    GeoUtil.cpp
    FieldCorner.cpp
    BotDetector.cpp
    AiDataLoader.cpp
    StaticDetector.cpp
    FieldModel.cpp
    LineSampler.cpp
    PointRenderer.cpp
    LineRenderer.cpp
    CircleRenderer.cpp
    TextRenderer.cpp
    FieldInit.cpp
    Transformer.cpp
)
set(augmSSL_HEADERS
    MainWindow.h
)

FIND_PACKAGE(Qt4 REQUIRED)
INCLUDE(${QT_USE_FILE})
ADD_DEFINITIONS(${QT_DEFINITIONS})
QT4_WRAP_CPP(augmSSL_HEADERS_MOC ${augmSSL_HEADERS})

include(FindProtobuf)
find_package( Protobuf REQUIRED )
include_directories(${PROTOBUF_INCLUDE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS augm_wrapper.proto augm_wrapper_container.proto)

add_executable( augmSSL MainWindow.cpp ${augmSSL_SOURCES} ${augmSSL_HEADERS_MOC} ${PROTO_SRCS} ${PROTO_HDRS} )
target_link_libraries( augmSSL ${OpenCV_LIBS} ${PROTOBUF_LIBRARY} ${Boost_LIBRARIES} ${QT_LIBRARIES} )

add_executable( transCalibration TransformationCalibration.cpp GeoUtil.cpp FieldCorner.cpp FieldModel.cpp )
target_link_libraries( transCalibration ${OpenCV_LIBS} )
