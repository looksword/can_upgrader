cmake_minimum_required(VERSION 3.16)

project(can_upgrader VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt5 REQUIRED COMPONENTS SerialBus Core)

add_executable(can_upgrader
    native_can_upgrader.h native_can_upgrader.cpp
    main.cpp
)

# 编译定义
target_compile_definitions(can_upgrader PRIVATE
    QT_DEPRECATED_WARNINGS
)

# 链接Qt库
target_link_libraries(can_upgrader PRIVATE
    Qt5::SerialBus
    Qt5::Core
)

if(WIN32)
    target_link_libraries(canfd PRIVATE
        DbgHelp
    )
endif()
