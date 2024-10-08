public class Simulation {
    public static void main(String[] args) {
        // Simulation parameters
        double deltaTime = 60.0; // Time step in seconds (e.g., 1 minute)
        double totalTime = 172800.0; // Total simulation time in seconds (e.g., 48 hours)
        int steps = (int) (totalTime / deltaTime);
    
        // Ambient temperature simulation parameters
        double baseAmbientTemp = 25.0; // Average ambient temperature (°C)
        double amplitude = 10.0;       // Amplitude of temperature fluctuation (°C)
        double period = 86400.0;       // Period of fluctuation (seconds) - e.g., 24 hours
        double omega = 2 * Math.PI / period; // Angular frequency
    
        // System parameters
        double tau = 600.0; // Thermal time constant (seconds)
        double K = 1000.0;  // Heating/Cooling coefficient (W per unit control input)
        double C = 5000.0;  // Thermal capacity (J/°C)
    
        // Controllers
        PIDController pid = new PIDController(0.5, 0.1, 0.05);
        MPCController mpc = new MPCController(10, deltaTime, tau, K, C);
        HybridController hybridController = new HybridController(pid, mpc, 1.0);
    
        // Initial conditions
        double currentTemp = 25.0; // Starting enclosure temperature (°C)
        double setpoint = 25.0;    // Desired enclosure temperature (°C)
    
        // Arrays to store simulation data (optional)
        double[] timeData = new double[steps];
        double[] enclosureTempData = new double[steps];
        double[] controlData = new double[steps];
        double[] ambientTempData = new double[steps];
    
        for (int i = 0; i < steps; i++) {
            double currentTime = i * deltaTime;
    
            // Calculate ambient temperature
            double T_ambient = baseAmbientTemp + amplitude * Math.sin(omega * currentTime);
    
            // Generate ambient temperature profile for MPC
            double[] ambientTempProfile = new double[mpc.predictionHorizon];
            for (int k = 0; k < mpc.predictionHorizon; k++) {
                double futureTime = currentTime + k * deltaTime;
                ambientTempProfile[k] = baseAmbientTemp + amplitude * Math.sin(omega * futureTime);
            }
    
            // Calculate control input
            double controlInput = hybridController.calculate(setpoint, currentTemp, deltaTime, ambientTempProfile);
    
            // Enforce control input limits
            controlInput = Math.max(-1.0, Math.min(1.0, controlInput));
    
            // Update enclosure temperature
            double heatTransfer = (T_ambient - currentTemp) * (C / tau);
            double controlEffect = K * controlInput;
            double tempChange = deltaTime * (heatTransfer + controlEffect) / C;
            currentTemp += tempChange;
    
            // Store data for analysis
            timeData[i] = currentTime;
            enclosureTempData[i] = currentTemp;
            controlData[i] = controlInput;
            ambientTempData[i] = T_ambient;
    
            // Log data (optional)
            System.out.printf(
                "Time: %6.0f s, Enclosure Temp: %6.2f °C, Ambient Temp: %6.2f °C, Control Input: %6.2f%n",
                currentTime, currentTemp, T_ambient, controlInput
            );
        }
    
        // Optional: Plotting the results or exporting data
    }
}
