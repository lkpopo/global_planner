//
// Created by 熊晨 on 2025/12/13.


#ifndef GPS_UTILS_H
#define GPS_UTILS_H

#include <cmath>
#include <string>
#include <iostream>

class GPSUtils {
public:
    // WGS84 椭球体参数
    static constexpr double WGS84_A = 6378137.0;            // 长半轴
    static constexpr double WGS84_F = 1.0 / 298.257223563;  // 扁率
    static constexpr double WGS84_B = WGS84_A * (1 - WGS84_F); // 短半轴
    static constexpr double k0 = 0.9996;                    // UTM 比例因子



    /**
     * @brief 将经纬度转换为 UTM 坐标 (包含高度透传)
     * @param lat 纬度 (度)
     * @param lon 经度 (度)
     * @param alt 高度 (米)
     * @return UTMResult 包含 x(东), y(北), z(高)
     */
    static global_planner::UTM_Location LatLonToUTM(double lat, double lon, double alt) {
        global_planner::UTM_Location result;
        //result.north = (lat >= 0);
        result.z = alt; // 【新增】高度直接透传，不需要投影计算

        // 1. 计算 UTM 区域号 (Zone)
        int zone = (int)((lon + 180.0) / 6.0) + 1;

        // 计算该区域的中央子午线经度 (Central Meridian)
        double lon0 = -183.0 + (zone * 6.0);

        // 角度转弧度
        double lat_rad = lat * M_PI / 180.0;
        double lon_rad = lon * M_PI / 180.0;
        double lon0_rad = lon0 * M_PI / 180.0;

        // 2. 椭球体计算中间变量
        double e = std::sqrt(1 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A)); // 第一偏心率
        double e2 = e * e;
        double e4 = e2 * e2;
        double e6 = e4 * e2;

        double N = WGS84_A / std::sqrt(1 - e2 * std::sin(lat_rad) * std::sin(lat_rad)); // 卯酉圈曲率半径
        double T = std::tan(lat_rad) * std::tan(lat_rad);
        double C = e2 * std::cos(lat_rad) * std::cos(lat_rad) / (1 - e2);
        double A = (lon_rad - lon0_rad) * std::cos(lat_rad);

        // 3. 计算子午线弧长 M
        double M = WGS84_A * ((1 - e2 / 4 - 3 * e4 / 64 - 5 * e6 / 256) * lat_rad
                            - (3 * e2 / 8 + 3 * e4 / 32 + 45 * e6 / 1024) * std::sin(2 * lat_rad)
                            + (15 * e4 / 256 + 45 * e6 / 1024) * std::sin(4 * lat_rad)
                            - (35 * e6 / 3072) * std::sin(6 * lat_rad));

        // 4. 计算 UTM 坐标 (x, y)
        // 东坐标 (Easting)
        result.x = k0 * N * (A + (1 - T + C) * A * A * A / 6
                            + (5 - 18 * T + T * T + 72 * C - 58 * e2) * A * A * A * A * A / 120)
                   + 500000.0; // 加上 500km 伪东偏移

        // 北坐标 (Northing)
        result.y = k0 * (M + N * std::tan(lat_rad) * (A * A / 2
                            + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24
                            + (61 - 58 * T + T * T + 600 * C - 330 * e2) * A * A * A * A * A * A / 720));

        // 如果是南半球，加上 10,000km 伪北偏移
        // if (!result.north) {
        //     result.y += 10000000.0;
        // }

        return result;
    }
};

#endif // GPS_UTILS_H
