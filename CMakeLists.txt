# 要求CMake至少3.16，低版本需升级或使用等效构建环境。
cmake_minimum_required(VERSION 3.16)
# 声明纯C项目及版本，不启用C++编译器。
project(scan_match VERSION 0.1.0 LANGUAGES C)
# 生成编译数据库，供IDE跳转、补全和静态检查使用。
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# 默认构建PC命令行回放程序；只需要库时可关闭。
option(SCAN_MATCH_BUILD_CLI "Build PC replay" ON)
# 默认关闭地址/未定义行为检查，测试时可显式开启。
option(SCAN_MATCH_SANITIZE "Enable ASan/UBSan" OFF)
# 测试默认不编译；review或修改核心后用ON启用CTest目标。
option(BUILD_TESTING "Build regression tests" OFF)
# 真实toflib源码路径保存到CMake缓存，可在命令行用-D覆盖。
set(SCAN_MATCH_TOFLIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../tof_match_tool/toflib" CACHE PATH "Actual toflib source directory")
# 引入CTest支持，根据BUILD_TESTING建立测试运行配置。
include(CTest)
# 接口库只传播编译/链接选项，本身不产生源码对象。
add_library(scan_match_options INTERFACE)
# 要求C99特性，供核心及其使用者继承。
target_compile_features(scan_match_options INTERFACE c_std_99)
# 只向GCC/Clang传它们支持的警告选项，避免其他编译器不认识。
if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    # 开启常用、额外和标准符合性警告，便于发现类型或声明问题。
    target_compile_options(scan_match_options INTERFACE -Wall -Wextra -Wpedantic)
# 结束当前构建条件分支。
endif()
# 只有显式打开内存检查才添加检查器，日常固件移植不带这些运行时。
if(SCAN_MATCH_SANITIZE)
    # 保留调试和栈帧，并启用ASan/UBSan插桩，便于定位越界与未定义行为。
    target_compile_options(scan_match_options INTERFACE -g -O1 -fsanitize=address,undefined -fno-omit-frame-pointer)
    # 链接时也启用检查器运行时，否则插桩对象无法正常链接。
    target_link_options(scan_match_options INTERFACE -fsanitize=address,undefined)
# 结束当前构建条件分支。
endif()
# 把算法核心单独构建为库，避免与PC文件读写或RTOS耦合。
add_library(scan_match_core core/scan_match.c)
# 公开核心头文件和真实toflib头路径，使用者继承类型声明。
target_include_directories(scan_match_core PUBLIC core "${SCAN_MATCH_TOFLIB_DIR}/inc")
# 核心继承统一选项并链接数学库m。
target_link_libraries(scan_match_core PUBLIC scan_match_options m)
# 禁用核心的编译器C扩展，保持接口实现接近标准C99。
set_target_properties(scan_match_core PROPERTIES C_EXTENSIONS OFF)
# 适配库包括零拷贝适配器与第四方法包装，后者源码位于真实toflib目录。
add_library(scan_match_toflib adapters/toflib_adapter.c "${SCAN_MATCH_TOFLIB_DIR}/src/tof_match_fourth.c")
# 向链接适配库的目标公开adapters头文件路径。
target_include_directories(scan_match_toflib PUBLIC adapters)
# 适配层依赖核心，PUBLIC使下游目标继承这项依赖。
target_link_libraries(scan_match_toflib PUBLIC scan_match_core)
# 仅在需要PC回放时建立命令行目标。
if(SCAN_MATCH_BUILD_CLI)
    # 以cli/main.c构建PC程序，输出JSON和可选CSV。
    add_executable(scan_match cli/main.c)
    # PC程序链接适配/核心库，不改变固件默认算法选择。
    target_link_libraries(scan_match PRIVATE scan_match_toflib)
# 结束当前构建条件分支。
endif()
# 以下目标用于验证，不作为默认交付固件源码路径的一部分。
if(BUILD_TESTING)
    # 构建核心已知变换、枚举/BnB一致性和异常输入测试。
    add_executable(test_core tests/test_core.c)
    # 核心测试只需要核心库，不依赖旧库RTOS桩。
    target_link_libraries(test_core PRIVATE scan_match_core)
    # 将核心程序注册为CTest用例，退出码非0即失败。
    add_test(NAME core COMMAND test_core)
    # 构建真实类型、地图布局、射线转换和缓存边界测试。
    add_executable(test_toflib_adapter tests/test_toflib_adapter.c)
    # 适配测试链接实际适配实现，不使用测试替身替换算法。
    target_link_libraries(test_toflib_adapter PRIVATE scan_match_toflib)
    # 注册适配层测试，可在Release中运行断言。
    add_test(NAME toflib_adapter COMMAND test_toflib_adapter)
    # 编译实际tof_gridmap.c；只替换PC缺失的RTOS/硬件接口，不替换插图逻辑。
    # 编译真实旧库匹配、插图及相关源码，PC缺失的板级函数由harness桩提供。
    add_library(toflib_integration STATIC
        # 实际匹配入口和插图实现，验证第四方法接入位置。
        "${SCAN_MATCH_TOFLIB_DIR}/src/tof_gridmap.c"
        # 旧流程依赖的地图细化实现，满足真实链接依赖。
        "${SCAN_MATCH_TOFLIB_DIR}/src/tof_map_thinning.c"
        # 旧库相关重定位依赖，保持实际组件可链接。
        "${SCAN_MATCH_TOFLIB_DIR}/src/tof_fullmap_reloc.c"
        # 仅替换PC没有的RTOS、锁和硬件接口，不替换匹配算法。
        "${SCAN_MATCH_TOFLIB_DIR}/../harness/stubs.c")
    # 在实际旧库源文件中启用第四方法分支；仍需运行时选择4才使用。
    target_compile_definitions(toflib_integration PRIVATE TOF_ENABLE_SCAN_MATCH4=1)
    # 只给PC集成目标加入RTOS/硬件桩头文件，不能复制到正式固件覆盖真实接口。
    target_include_directories(toflib_integration PRIVATE "${SCAN_MATCH_TOFLIB_DIR}/../harness/stubs")
    # 旧库集成目标链接第四方法适配库。
    target_link_libraries(toflib_integration PUBLIC scan_match_toflib)
    # 抑制部分旧库未使用项警告；不为注释任务修改旧算法代码。
    target_compile_options(toflib_integration PRIVATE -Wno-unused-parameter -Wno-unused-variable -Wno-unused-function -Wno-pedantic)
    # 构建真实tryMatchEx入口的第四方法测试。
    add_executable(test_fourth tests/test_fourth.c)
    # 使用真实旧库加PC板级桩，覆盖输出评分及旧应用路径。
    target_link_libraries(test_fourth PRIVATE toflib_integration)
    # 注册第四方法集成用例。
    add_test(NAME fourth COMMAND test_fourth)
    # 仅在需要PC回放时建立命令行目标。
    if(SCAN_MATCH_BUILD_CLI)
        # Python测试需要解释器；不构建CLI时不要求Python测试环境。
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        # 用cmake -E env给Python传本次构建的C程序绝对路径。
        add_test(NAME pipeline COMMAND "${CMAKE_COMMAND}" -E env
            # 生成器表达式自动取得当前配置的目标文件，避免脚本运行到旧二进制。
            "SCAN_MATCH_EXECUTABLE=$<TARGET_FILE:scan_match>"
            # 按unittest发现tests下的test_*.py，详细输出每条验证结果。
            "${Python3_EXECUTABLE}" -m unittest discover -s tests -p "test_*.py" -v)
        # 把Python测试工作目录设为工程根，保证相对测试发现路径有效。
        set_tests_properties(pipeline PROPERTIES WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
    # 结束当前构建条件分支。
    endif()
# 结束当前构建条件分支。
endif()
