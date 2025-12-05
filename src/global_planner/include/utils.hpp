#pragma once
#include <cmath>
#include <Eigen/Dense>

// 用于坐标系转换的常量
static constexpr double WGS84_A = 6378137.0;                // 长半轴
static constexpr double WGS84_F = 1.0 / 298.257223563;      // 扁率
static constexpr double WGS84_E2 = WGS84_F * (2 - WGS84_F); // 偏心率平方
static constexpr double UTM_K0 = 0.9996;

int lonToUTMZone(double lon)
{
    return int((lon + 180.0) / 6.0) + 1;
}

Eigen::Vector3d gpsToUtm(double lat, double lon, double alt, int &utm_zone_)
{
    // 度 → 弧度
    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    // 确定 UTM  Zone
    int zone = lonToUTMZone(lon);
    utm_zone_ = zone;

    double lon0 = (zone - 1) * 6 - 180 + 3; // 中央经线
    double lon0_rad = lon0 * M_PI / 180.0;

    double N = WGS84_A / sqrt(1 - WGS84_E2 * sin(lat_rad) * sin(lat_rad));
    double T = tan(lat_rad) * tan(lat_rad);
    double C = WGS84_E2 / (1 - WGS84_E2) * cos(lat_rad) * cos(lat_rad);
    double A = cos(lat_rad) * (lon_rad - lon0_rad);

    double M = WGS84_A * ((1 - WGS84_E2 / 4 - 3 * WGS84_E2 * WGS84_E2 / 64 - 5 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 256) * lat_rad - (3 * WGS84_E2 / 8 + 3 * WGS84_E2 * WGS84_E2 / 32 + 45 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 1024) * sin(2 * lat_rad) + (15 * WGS84_E2 * WGS84_E2 / 256 + 45 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 1024) * sin(4 * lat_rad) - (35 * WGS84_E2 * WGS84_E2 * WGS84_E2 / 3072) * sin(6 * lat_rad));

    double x = UTM_K0 * N * (A + (1 - T + C) * pow(A, 3) / 6 + (5 - 18 * T + T * T + 72 * C - 58 * WGS84_E2) * pow(A, 5) / 120) + 500000.0;

    double y = UTM_K0 * (M + N * tan(lat_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * pow(A, 4) / 24 + (61 - 58 * T + T * T + 600 * C - 330 * WGS84_E2) * pow(A, 6) / 720));

    if (lat < 0)
        y += 10000000.0;

    return Eigen::Vector3d(x, y, alt);
}