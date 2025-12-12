#include "WaylineManager.h"
#include <iostream>
#include <sstream>
#include <vector>

// Include the dependencies you will download
#include "miniz.h"
#include "pugixml.hpp"

WaylineManager::WaylineManager(const std::string& kmz_path) : kmz_filepath(kmz_path) {}

WaylineManager::~WaylineManager() {}

void WaylineManager::setError(const std::string& msg) {
    error_message = msg;
    std::cerr << "Error: " << error_message << std::endl;
}

bool WaylineManager::load() {
    waypoints.clear();
    if (!decompress_kmz()) {
        return false;
    }
    if (!parse_wpml()) {
        return false;
    }
    return true;
}

std::vector<global_planner::waypoint> WaylineManager::getWaypoints() const {
    return waypoints;
}

bool WaylineManager::decompress_kmz() {
    mz_zip_archive zip_archive;
    memset(&zip_archive, 0, sizeof(zip_archive));

    if (!mz_zip_reader_init_file(&zip_archive, kmz_filepath.c_str(), 0)) {
        setError("Could not initialize KMZ/ZIP reader for file: " + kmz_filepath);
        return false;
    }

    const char* target_file = "wpmz/waylines.wpml";
    int file_index = mz_zip_reader_locate_file(&zip_archive, target_file, NULL, 0);

    if (file_index < 0) {
        setError("Could not find '" + std::string(target_file) + "' in the KMZ archive.");
        mz_zip_reader_end(&zip_archive);
        return false;
    }

    mz_zip_archive_file_stat file_stat;
    if (!mz_zip_reader_file_stat(&zip_archive, file_index, &file_stat)) {
        setError("Could not get file stats for '" + std::string(target_file) + "'.");
        mz_zip_reader_end(&zip_archive);
        return false;
    }

    size_t uncompressed_size = (size_t)file_stat.m_uncomp_size;
    wpml_buffer.resize(uncompressed_size);

    if (!mz_zip_reader_extract_to_mem(&zip_archive, file_index, &wpml_buffer[0], uncompressed_size, 0)) {
        setError("Failed to decompress '" + std::string(target_file) + "'.");
        mz_zip_reader_end(&zip_archive);
        return false;
    }

    mz_zip_reader_end(&zip_archive);
    return true;
}

bool WaylineManager::parse_wpml() {
    if (wpml_buffer.empty()) {
        setError("WPML buffer is empty, cannot parse.");
        return false;
    }

    pugi::xml_document doc;
    // Add a null terminator for safety, as load_buffer expects a C-string
    wpml_buffer.push_back('\0');
    pugi::xml_parse_result result = doc.load_buffer(wpml_buffer.data(), wpml_buffer.size());

    if (!result) {
        setError("Error parsing XML buffer: " + std::string(result.description()));
        return false;
    }

    pugi::xml_node folder = doc.child("kml").child("Document").child("Folder");
    if (!folder) {
        setError("Could not find required XML structure: kml/Document/Folder");
        return false;
    }

    for (pugi::xml_node placemark = folder.child("Placemark"); placemark; placemark = placemark.next_sibling("Placemark")) {
        global_planner::waypoint wp;

        // Location data
        pugi::xml_node coordinates_node = placemark.child("Point").child("coordinates");
        if (coordinates_node) {
            std::stringstream ss(coordinates_node.text().get());
            std::string lon_str, lat_str;
            std::getline(ss, lon_str, ',');
            std::getline(ss, lat_str, ',');
            try {
                wp.location.lo = std::stod(lon_str);
                wp.location.la = std::stod(lat_str);
            } catch (...) { /* ignore conversion errors */ }
        }
        wp.location.al = placemark.child("wpml:executeHeight").text().as_double();

        // UAV Attitude
        wp.attitude.yaw = placemark.child("wpml:waypointHeadingParam").child("wpml:waypointHeadingAngle").text().as_double();
        // UAV Roll and Pitch are not available in the file.

        // Gimbal Attitude
        pugi::xml_node gimbal_param = placemark.child("wpml:waypointGimbalHeadingParam");
        wp.gimbal.pitch = gimbal_param.child("wpml:waypointGimbalPitchAngle").text().as_double();
        wp.gimbal.yaw = gimbal_param.child("wpml:waypointGimbalYawAngle").text().as_double();
        
        // Fallback for gimbal data from action groups
        pugi::xml_node action_param = placemark.child("wpml:actionGroup").child("wpml:action").child("wpml:actionActuatorFuncParam");
        if (action_param) {
            wp.gimbal.roll = action_param.child("wpml:gimbalRollRotateAngle").text().as_double(wp.gimbal.roll);
            wp.gimbal.pitch = action_param.child("wpml:gimbalPitchRotateAngle").text().as_double(wp.gimbal.pitch);
            wp.gimbal.yaw = action_param.child("wpml:gimbalYawRotateAngle").text().as_double(wp.gimbal.yaw);
        }
        
        waypoints.push_back(wp);
    }
    return true;
}
