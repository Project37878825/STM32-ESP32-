# 设置支持的芯片类型
set(SUPPORTED_TARGETS esp32)

# 引入ESP-IDF项目构建系统
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sample_project)
