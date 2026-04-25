# MovementPath

一个使用 Open CASCADE (OCCT) 创建简单几何并导出为 STEP 文件的 C++ 示例项目。

项目概述
- 使用 C++17 和 CMake（最低版本 3.19）构建。
- 演示如何使用 OCCT 的 BRepPrimAPI 创建一个 box 并将其导出为 STEP（src/main.cpp）。

先决条件
- CMake >= 3.19
- 支持 C++17 的编译器（在 Windows 上通常使用 MSVC）
- Ninja（或其它 CMake 生成器）
- Open CASCADE Technology (OCCT) 库及其头文件、库和 DLL。项目包含了一个 cmake 脚本（src/cmake/occt_setup_install.cmake）用于设置 OCCT 路径。

构建（示例，Windows / 命令行）
1. 在仓库根目录执行：
   cmake -S . -B build -G "Ninja"
2. 构建：
   cmake --build build

构建产物和运行
- 可执行文件为 MovementPath（在 build 目录内）。
- 运行程序会在当前工作目录生成 `box.step`，这是 main.cpp 中创建的简单立方体的 STEP 文件。

项目结构（重要文件）
- CMakeLists.txt — 根 CMake 配置
- src/CMakeLists.txt — 二级 CMake 配置，包含 OCCT 设置
- src/main.cpp — 演示代码：创建 box 并导出 STEP
- src/cmake/occt_setup_install.cmake — OCCT 的查找/配置脚本
- depends/occt — （可选）仓库内的 OCCT 头文件/二进制依赖目录

注意事项
- 确保 OCCT 的库和 DLL 在 CMake 配置中能被正确找到。默认脚本会将所需 DLL/PDB 复制到构建输出目录。
- 如果使用不同的 OCCT 安装路径，请编辑或通过 CMake 选项覆盖 cmake/occt_setup_install.cmake 中的变量。

贡献及许可证
- 欢迎提交 issue 与 PR。
- 当前仓库未包含许可证文件，请在需要时添加 LICENSE。

作者
- 本项目演示代码由仓库维护者提供。