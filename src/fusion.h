// fusion.h - combines the latest LIDAR, camera and GPS readings into one picture of the world
// and decides what the car should do.
//
// In the full OS project this is the job of the AI processor (the consumer thread): it will take
// readings out of the shared buffer instead of receiving them directly.
#pragma once

#include <cstdint>

#include "sensors/camera.h"
#include "sensors/gps.h"
#include "sensors/lidar.h"

enum class Decision { Cruise, Slow, Brake };

const char* toString(Decision d);   // "CRUISE", "SLOW", "BRAKE"

// BRAKE if an obstacle is closer than 10 m in front, SLOW if closer than 25 m, otherwise CRUISE.
Decision decide(const sensors::LidarScan& lidar);

// The newest reading from every sensor, plus the decision made from them.
struct FusedState {
    double timeSec = 0;
    sensors::LidarScan lidar;
    sensors::CameraFrame camera;
    sensors::GpsFix gps;
    Decision decision = Decision::Cruise;
};

// How much data each sensor has produced so far.
struct DataCounters {
    uint32_t lidarScans = 0, cameraFrames = 0, gpsFixes = 0;
    uint64_t lidarBytes = 0, cameraBytes = 0, gpsBytes = 0;
};
