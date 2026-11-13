# 第三方依赖说明

FamilyCore 自有代码按 [MIT](LICENSE) 发布。Qt、编译器运行库和上游模块仍适用各自许可；本仓库不附带它们的源码、SDK 或二进制。

- Qt 使用的具体模块、许可及归属以所选 Qt 发行包为准，见 [Qt 许可](https://doc.qt.io/qt-6/licensing.html) 和 [Qt 开源义务](https://www.qt.io/development/open-source-lgpl-obligations)。本项目的 MIT 不替代 Qt 授权。
- OpenMP（FamilyCore 的并行族算法）由编译器提供；若随应用交付其运行库，保留相应 GCC/MSVC/LLVM 发行包要求的声明。
- 上游自有模块使用 MIT，需保留各自 LICENSE：[Algorithm](https://github.com/Lateef-ZHOU/Algorithm), [DeviceCore](https://github.com/Lateef-ZHOU/DeviceCore)。启用可选 QML 参考组件时还使用 [UICore](https://github.com/Lateef-ZHOU/UICore) 和 [ChartCore](https://github.com/Lateef-ZHOU/ChartCore)，同样须保留各自 MIT 声明。

发布本仓库源码与分发含 Qt/其他依赖的可执行文件、容器或固件是不同场景。产品发布者应依据实际交付内容准备第三方许可文本、版权归属和适用的源码/重新链接材料；本说明和上游链接不代替这些材料。

默认仅使用 HandleCore 基础目标，不引入 libserialport、libpcap/Npcap。应用自行启用这些可选模块时，还须遵循 HandleCore 的第三方依赖说明及所用发行包许可。

FamilyModels 轻量目标不依赖 Qt/OpenMP，只通过 Algorithm 使用标准库与 Boost.CircularBuffer（Boost Software License 1.0）；完整族运行目标和可选界面依赖按上文保留。
