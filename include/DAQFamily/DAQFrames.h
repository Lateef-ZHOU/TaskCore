#pragma once
#include <optional>

#include "Matrix.h"
#include "DataArrays.h"

// DAQ 侧对绘图库容器的具体化：风廓线帧 = 带墙钟时间的一维风矢量数组。
// 容器与轴由绘图库提供，这里只把元素类型定下来，不另造数据布局。
template<MappingLike Range>
using WindProfileFrame = TimedArray1D<Range, std::optional<LinAlg::Vec3>>;
