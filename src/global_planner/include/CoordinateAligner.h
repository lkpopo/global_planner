//
// Created by 熊晨 on 2025/12/13.
//

#ifndef COORDINATE_ALIGNER_H
#define COORDINATE_ALIGNER_H

#include "GPSUtils.h" // 引用之前给你的 GPSUtils
#include <mutex>

class CoordinateAligner {
public:
    CoordinateAligner() : is_aligned_(false) {}

    // 重置状态（每次重新起飞或任务重置时调用）
    void Reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        is_aligned_ = false;
        origin_utm_ = {0, 0, 0};
    }

    /**
     * @brief 初始化对齐 (只在第一次调用时生效)
     * @param lat, lon, alt  当前 GPS 读数
     * @param slam_x, slam_y, slam_z 当前 SLAM 读数
     * @return true=刚刚完成初始化, false=已经是初始化状态
     */
    bool InitOrigin(double lat, double lon, double alt,
                    double slam_x, double slam_y, double slam_z) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (is_aligned_) return false; // 已经对齐过，不再重复对齐

        // 1. 将当前 GPS 转为 UTM
        global_planner::UTM_Location current_utm_gps = GPSUtils::LatLonToUTM(lat, lon, alt);

        // UTM X (东) = 当前UTM东 - SLAM的东向分量(即 slam_y)
        origin_utm_.x = current_utm_gps.x - slam_y;

        // UTM Y (北) = 当前UTM北 - SLAM的北向分量(即 slam_x)
        origin_utm_.y = current_utm_gps.y - slam_x;

        origin_utm_.z = current_utm_gps.z - slam_z;

        is_aligned_ = true;
        return true;
    }

    /**
     * @brief 根据实时 SLAM 偏移量，计算实时 UTM
     * @param slam_x, slam_y, slam_z 实时 SLAM 坐标
     * @return 计算出的实时 UTM 坐标
     */
    global_planner::UTM_Location GetRealTimeUTM(double slam_x, double slam_y, double slam_z) {
        std::lock_guard<std::mutex> lock(mutex_);
        global_planner::UTM_Location result;

        if (!is_aligned_) {
            // 如果还没对齐，返回空或错误码，这里暂时返回0
            return result;
        }

        // 正向推算：实时 UTM = 锚点 + 实时 SLAM 偏移
        // SLAM Y 加到 UTM X(东), SLAM X 加到 UTM Y(北)

        result.x = origin_utm_.x + slam_y; // East
        result.y = origin_utm_.y + slam_x; // North
        result.z = origin_utm_.z + slam_z; // Up

        return result;
    }

    bool IsAligned() const { return is_aligned_; }

private:
    std::mutex mutex_;
    bool is_aligned_;
    global_planner::UTM_Location origin_utm_; // 记录 SLAM (0,0,0) 对应的 UTM 坐标
};

#endif // COORDINATE_ALIGNER_H