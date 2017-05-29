#include "PID.h"

#include <iostream>
#include <limits>

using namespace std;

/*
 * TODO: Complete the PID class.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
    K[0] = Kp;
    K[1] = Ki;
    K[2] = Kd;
    i_error = 0.;
    p_error = 0.;
    d_error = 0.;

    dK[0] = 0.05;
    dK[1] = 0.0005;
    dK[2] = 0.5;

    // twiddling state
    error = 0.;
    iteration = 0;
    twiddling_index = -1;
}

void PID::UpdateError(double cte)
{
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    error += cte * cte;
    ++iteration;
}

void PID::Twiddle()
{
    if (iteration < 1400)
        return;

    std::cout << "Twiddle " << (dK[0] + dK[1] + dK[2]) << " index =" << twiddling_index << " best_error = " << best_error << " error = " << error << "\n";
    std::cout << " K: " << K[0] << " " << K[1] << " " << K[2] << "\n";
    std::cout << "dK: " << dK[0] << " " << dK[1] << " " << dK[2] << "\n";

    if (dK[0] + dK[1] + dK[2] < 1e-4)
        return;

    if (twiddling_index < 0)
    { // Init twiddling
        best_error = error;
        //
        if (++twiddling_index == 0)
            K[twiddling_index/2] += dK[twiddling_index/2];
    }
    else
    {
        if (error < best_error)
        {
            best_error = error;
            dK[twiddling_index / 2] *= 1.1;
            twiddling_index = (twiddling_index + (twiddling_index % 2 == 0 ? 2 : 1)) % 6;
            K[twiddling_index/2] += dK[twiddling_index/2];
        }
        else
        {
            if (twiddling_index % 2 == 0)
            {
                twiddling_index++;
                K[twiddling_index/2] -= 2 * dK[twiddling_index/2];
            }
            else
            {
                K[twiddling_index/2] += dK[twiddling_index/2];
                dK[twiddling_index / 2] *= 0.9;
                twiddling_index = (twiddling_index + 1) % 6;
                K[twiddling_index/2] += dK[twiddling_index/2];
            }
        }
    }

    std::cout << "Updated to index = " << twiddling_index << "\n";
    std::cout << " K: " << K[0] << " " << K[1] << " " << K[2] << "\n";
    std::cout << "dK: " << dK[0] << " " << dK[1] << " " << dK[2] << "\n";

    error = 0;
    iteration = 0;
}

double PID::SteeringValue()
{
    std::cout << p_error << " " << i_error << " " << d_error << " " << (K[0] * p_error + K[1] * i_error + K[2] * d_error) << "    iteration = " << iteration << "  error = " <<  error << "\n";
    return K[0] * p_error + K[1] * i_error + K[2] * d_error;
}
