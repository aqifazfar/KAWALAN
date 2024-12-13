#include <iostream>
#include <cstdint>
#include <cmath>

typedef struct
{
    double setpoint;
    double input;
    double Kp;
    double Ki;
    double Kd;
    double dt;
    double integral;
    double de;
    double error0;
    double error1;

} PID;

typedef struct
{
    double Vehicle_Velocity;
    double Vehicle_Heading;
    double Waypoint_Heading;
    double *Waypoints;
    double Delta_Distance;
    double Distance;
    std::uint32_t n;

} Autopilot;

typedef struct
{
    std::uint8_t Start : 1;
    std::uint8_t Interrupt : 1;
} Signal;

namespace kawalan
{
    double kawalan::PID_Controller(PID *pid);

    void kawalan::Auto_Pilot(PID *pid, Autopilot *a, Signal *signal);

} // namespace kawalan
