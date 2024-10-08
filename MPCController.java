public class MPCController {
    public int predictionHorizon;
    private double deltaTime;
    private double tau;
    private double K;
    private double C;
    
    public MPCController(int predictionHorizon, double deltaTime, double tau, double K, double C) {
        this.predictionHorizon = predictionHorizon;
        this.deltaTime = deltaTime;
        this.tau = tau;
        this.K = K;
        this.C = C;
    }
    
    public double calculate(double setpoint, double currentTemp, double[] ambientTempProfile) {
        double minCost = Double.MAX_VALUE;
        double optimalControl = 0.0;
    
        // Discretize control inputs within allowable range
        for (double controlInput = -1.0; controlInput <= 1.0; controlInput += 0.1) {
            double cost = 0.0;
            double temp = currentTemp;
    
            // Predict future temperatures over the horizon
            for (int k = 0; k < predictionHorizon; k++) {
                double T_ambient_future = ambientTempProfile[k];
                double heatTransfer = (T_ambient_future - temp) * (C / tau);
                double controlEffect = K * controlInput;
                double tempChange = deltaTime * (heatTransfer + controlEffect) / C;
                temp += tempChange;
                cost += Math.pow(setpoint - temp, 2); // Sum of squared errors
            }
    
            // Find control input with minimal cost
            if (cost < minCost) {
                minCost = cost;
                optimalControl = controlInput;
            }
        }
    
        return optimalControl;
    }
}
