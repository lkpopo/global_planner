#ifndef WAYLINE_MANAGER_H
#define WAYLINE_MANAGER_H

#include <string>
#include <vector>

#include "planner.h"

class WaylineManager {
public:
    // Constructor expects the path to the .kmz file
    WaylineManager(const std::string& kmz_path);
    ~WaylineManager();

    // The main method to load and parse the file.
    // Returns true on success, false on failure.
    bool load();

    // Returns the parsed waypoints.
    std::vector<global_planner::waypoint> getWaypoints() const;

private:
    std::string kmz_filepath;
    std::vector<char> wpml_buffer; // To store the unzipped file in memory
    std::vector<global_planner::waypoint> waypoints;
    std::string error_message;

    // Private methods for internal logic
    bool decompress_kmz();
    bool parse_wpml();

    // Sets the internal error message
    void setError(const std::string& msg);
};

#endif // WAYLINE_MANAGER_H
