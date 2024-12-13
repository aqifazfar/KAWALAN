#include "KAWALAN/Kawalan.h"

double kawalan::PID_Controller(PID *pid)
{
	double output;

	pid->error1 = pid->input - pid->setpoint;

	pid->integral = pid->integral + pid->error1 * pid->dt;

	pid->de = pid->error1 - pid->error0;

	output = pid->Kp * pid->error1 + pid->Ki * pid->integral + pid->Kd * pid->de / pid->dt;

	pid->error0 = pid->error1;

	return output;
}

void kawalan::Auto_Pilot(PID *pid, Autopilot *a, Signal *signal)
{
	if (signal->Start >= 1)
	{
		if (signal->Interrupt >= 1)
		{
			return;
		}

		if (a->Delta_Distance <= 5)
		{
			// Calculate new bearing and distance

			double Delta_Latitude = a->Waypoints[2 * a->n + 2] - a->Waypoints[2 * a->n];

			double Delta_Longitude = a->Waypoints[2 * a->n + 3] - a->Waypoints[2 * a->n + 1];

			double clat0 = cos(a->Waypoints[2 * a->n]);

			double clat1 = cos(a->Waypoints[2 * a->n + 2]);

			double b = sin(Delta_Latitude / 2) * sin(Delta_Latitude / 2) + clat0 * clat1 * sin(Delta_Longitude / 2) * sin(Delta_Longitude / 2);

			a->Distance = 6371000.00f * (2 * atan2(sqrt(b), sqrt(1 - b)));

			double Bearing = atan2(sin(Delta_Longitude) * clat1, clat0 * sin(a->Waypoints[2 * a->n + 2]) - sin(a->Waypoints[2 * a->n]) * clat1 * cos(Delta_Longitude));

			a->n += 1;
		}
	}
}
