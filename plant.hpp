#pragma once
// Your model of the actuator, reconstructed from the decoded CSVs. This is
// the Part B deliverable, alongside your written notes.
//
// Implement step(): given a commanded velocity and a timestep, return the
// measured output angle. The placeholder below is a bare integrator with
// gain 1 -- NOT the real actuator. Replace it with what the data shows
// (dynamics, gain, any nonlinearity, any lag), or the harness proves nothing.

#include <cmath>
#include <deque>

struct Plant {
    static constexpr double gain        = 1.3;
    static constexpr double lagTimeStep = 0.195;
    static constexpr double gapwidthDB  = 4.0;

    double rate     = 0.0;
    double drivePos = 0.0;
    double angle = gapwidthDB / 8.0;

    double step(double u_cmd, double dt) {
        rate += (gain * u_cmd - rate) * (dt / lagTimeStep);
        drivePos += rate * dt;

        double half = gapwidthDB / 2.0;
        if (drivePos - angle > half) {
            angle = drivePos - half;
        } else if (drivePos - angle < -half) {
            angle = drivePos + half;
        }
        return std::round(angle / 0.1) * 0.1;
    }

    void reset() { rate = 0.0; drivePos = 0.0; angle = 0.0; }
};
