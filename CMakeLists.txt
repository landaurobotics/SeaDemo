cmake_minimum_required(VERSION 3.10)

project(SeaProjectApp LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)

INCLUDE_DIRECTORIES(/opt/etherlab/include)
LINK_DIRECTORIES(/opt/etherlab/lib)

INCLUDE_DIRECTORIES(src)

find_package(Qt6 COMPONENTS Gui Qml Quick)
qt_add_executable(SeaProjectApp src/main.cpp src/EcatConfig.cpp src/EcatConfig.h src/Backend.cpp src/Backend.h)
#qt_add_executable(SeaProjectApp src/main.cpp)


qt_add_resources(SeaProjectApp "configuration"
    PREFIX "/"
    FILES
        qtquickcontrols2.conf
)

target_link_libraries(SeaProjectApp PRIVATE
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    Qt${QT_VERSION_MAJOR}::Quick
    Qt${QT_VERSION_MAJOR}::Qml
    ethercat
    pthread
)

include(${CMAKE_CURRENT_SOURCE_DIR}/qmlmodules)
