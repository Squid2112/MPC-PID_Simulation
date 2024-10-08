public class HybridController {
    private PIDController pidController;
    private MPCController mpcController;
    private double switchThreshold;
    
    public HybridController(PIDController pidController, MPCController mpcController, double switchThreshold) {
        this.pidController = pidController;
        this.mpcController = mpcController;
        this.switchThreshold = switchThreshold;
    }
    
    public double calculate(double setpoint, double measurement, double deltaTime, double[] ambientTempProfile) {
        double error = Math.abs(setpoint - measurement);
    
        if (error > switchThreshold) {
            // Use MPC when error exceeds threshold
            return mpcController.calculate(setpoint, measurement, ambientTempProfile);
        } else {
            // Use PID for small errors
            return pidController.calculate(setpoint, measurement, deltaTime);
        }
    }
}
