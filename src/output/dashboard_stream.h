// dashboard_stream.h - sends the live data to the browser dashboard (tools/dashboard.py).
//
// It prints one line of JSON text per LIDAR scan to standard output; dashboard.py reads those lines.
// Camera pictures are too big for that, so each one is written to <outDir>/live/live_0 or live_1
// (in turn, so the dashboard never reads a half-written file) and the JSON line says which one.
#pragma once

#include <ostream>
#include <string>

#include "../fusion.h"
#include "../world.h"

class DashboardStream {
public:
    DashboardStream(std::ostream& out, const std::string& outDir);

    void sendWorld(const world::World& world);   // once at the start: road and obstacles for the map
    void saveLiveImage(const sensors::CameraFrame& frame);
    void sendState(const FusedState& state, const DataCounters& counters, bool usingPhotos);
    void sendEnd();
    bool connected() const { return (bool)out_; }   // false once the dashboard has closed

private:
    std::ostream& out_;
    std::string liveDir_, liveImagePath_;
};
