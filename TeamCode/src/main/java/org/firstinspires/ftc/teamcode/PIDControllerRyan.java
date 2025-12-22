package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDControllerRyan {
    private double Kp;
    private double Ki;
    private double Kd;
    private double integralSumLimit;
    private double lastTarget = 0;
    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime PIDtime = new ElapsedTime();
    private DcMotor motor; // Store the motor (now DcMotor)

    // Constructors
    public PIDControllerRyan(double kp, double ki, double kd, double integralSumLimit, DcMotor motor) {
        this.Kp = kp;
        this.Ki = ki;
        this.Kd = kd;
        this.integralSumLimit = integralSumLimit;
        this.motor = motor; // Assign the motor
    }
    public PIDControllerRyan(double kp, double ki, double kd, DcMotor motor) {
        this(kp, ki, kd, 3, motor); // Default integral sum limit
    }

    public PIDControllerRyan(double kp, double ki, DcMotor motor) {
        this(kp, ki, 0, 3, motor); // Default kd and integral sum limit
    }

    public PIDControllerRyan(DcMotor motor) {
        //default values, can be set later using setters
        this(0,0,0,0,motor);
    }

    // Setters for tuning
    public void setKp(double kp) {
        this.Kp = kp;
    }

    public void setKi(double ki) {
        this.Ki = ki;
    }

    public void setKd(double kd) {
        this.Kd = kd;
    }

    public void setIntegralSumLimit(double limit){ this.integralSumLimit = limit; }
    // Reset the integral sum and last error (useful when changing target significantly)
    public void reset() {
        integralSum = 0;
        lastError = 0;
        PIDtime.reset();
    }

    public double update(int target) { // Simplified update method
        return update(target, motor.getCurrentPosition());
    }

    public double update(int target, int current) {
        double error = target - current;
        double seconds = PIDtime.seconds();
        double p = Kp * error;

        // Integral term
        integralSum += error * seconds;
        // Clamp integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        } else if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        double i = Ki * integralSum;


        // Derivative term
        double derivative = (error - lastError) / seconds;
        double d = Kd * derivative;

        // Reset integral sum if target changes.  Essential for stability.
        if (lastTarget != target) {
            integralSum = 0;
        }

        PIDtime.reset();
        lastError = error;
        lastTarget = target;

        double power = p + i + d;

        // Output clamping and deadband
        if (power > 0.8) {
            power = 0.8;
        } else if (power < -.8) {
            power = -.8;
        } else if ((power < 0.01 && power > 0) || (power > -0.01 && power < 0)) {
            power = 0;
        }

        //rounding for telemetry
        int n = 3;
        power = Math.round(power * Math.pow(10, n))/Math.pow(10, n); //round to n decimal places

        return power;
    }


    // Optionally, add getters if you need to access the PID parameters externally
    public double getKp() { return Kp; }
    public double getKi() { return Ki; }
    public double getKd() { return Kd; }
    public double getIntegralSum() { return integralSum; }
    public double getLastError() { return lastError; }
    public DcMotor getMotor() {return motor;}
}