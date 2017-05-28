#ifndef PID_H
#define PID_H

class PID {
public:
    /*
     * Errors
     */
    double p_error;
    double i_error;
    double d_error;
    double error;
    double best_error;
    int iteration;
    int twiddling_index;

    /*
     * Coefficients
     */
    double K[3];
    double dK[3];

    /*
     * Constructor
     */
    PID();

    /*
     * Destructor.
     */
    virtual ~PID();

    /*
     * Initialize PID.
     */
    void Init(double Kp, double Ki, double Kd);

    /*
     * Update the PID error variables given cross track error.
     */
    void UpdateError(double cte);

    /*
     * Calculate the total PID error.
     */
    double SteeringValue();

    void Twiddle();
};

#endif /* PID_H */
