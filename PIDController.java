public class PIDController {
    private double kp;
    private double ki;
    private double kd;
    private double integral;
    private double previousError;

    public PIDController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
        this.integral = 0.0;
        this.previousError = 0.0;
    }

    public double calculate(double setpoint, double measurement, double deltaTime) {
        double error = setpoint - measurement;
        integral += error * deltaTime;
        double derivative = (error - previousError) / deltaTime;
        previousError = error;

        double output = kp * error + ki * integral + kd * derivative;
        return output;
    }
}
