cmake_minimum_required(VERSION 3.16)
project(lib-mapf)

# 소스 코드 자동 수집
file(GLOB SRCS "./src/*.cpp")

# 정적 라이브러리 생성
add_library(lib-mapf STATIC ${SRCS})

# define 설정: map 파일 디렉토리 상수
add_definitions(-D_MAPDIR_="${CMAKE_CURRENT_LIST_DIR}/../map/")

# 최적화 및 컴파일 옵션
if(CPU STREQUAL "M1")
    message("Configuring grid-pathfinding for M1 Mac.")
    target_compile_options(lib-mapf PUBLIC -O3 -Wall -mtune=native -mcpu=apple-m1)
else()
    target_compile_options(lib-mapf PUBLIC -O3 -Wall -mtune=native -march=native)
endif()

# C++17 사용
target_compile_features(lib-mapf PUBLIC cxx_std_17)

# include 디렉토리 설정
target_include_directories(lib-mapf
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include         # 예: linear_interpolator.hpp 포함
)

# 외부 라이브러리(graph) 포함
add_subdirectory(../third_party/grid-pathfinding/graph ./graph)
target_link_libraries(lib-mapf lib-graph)
