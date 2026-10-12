#include "commons.h"

// 设置时间，包括秒和纳秒部分
void PoseWithTime::setTime(int32_t _sec, uint32_t _nsec)
{
    // 保存秒部分
    sec = _sec;
    // 保存纳秒部分
    nsec = _nsec;
    // 将时间转换为秒数（带小数）
    second = static_cast<double>(sec) + static_cast<double>(nsec) / 1e9;
}