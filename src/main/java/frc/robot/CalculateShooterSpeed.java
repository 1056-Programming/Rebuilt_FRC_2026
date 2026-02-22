package frc.robot; 

public class CalculateShooterSpeed {
        
    public static double[] calculateOptimalShot(double targetDistanceFeet, double targetHeightFeet) {
        // Validate input
        if (targetDistanceFeet <= 0 || targetHeightFeet <= 0) {
            return new double[]{0, 0, 0, 0, 0};
        }
        
        // Find optimal combination of angle and velocity
        double[] optimalParams = findOptimalTrajectory(targetDistanceFeet, targetHeightFeet);
        double optimalAngle = optimalParams[0];
        double requiredVelocity = optimalParams[1];
        double requiredBallBackspinRPS = optimalParams[2]; // Ball spin in RPS
        
        // Convert to wheel RPS requirements
        double[] wheelRPS = calculateWheelRPS(requiredVelocity, requiredBallBackspinRPS);
        
        return new double[]{
            wheelRPS[0],              // Launcher wheel RPS
            wheelRPS[1],              // Backspin wheel RPS (negative for reverse direction)
            optimalAngle,              // Launch angle
            requiredVelocity,          // Exit velocity (ft/s)
            requiredBallBackspinRPS    // Ball backspin RPS
        };
    }
    
    /**
     * Find optimal trajectory using physics simulation
     */
    private static double[] findOptimalTrajectory(double distance, double targetHeight) {
        double bestAngle = 45.0;
        double bestVelocity = 0;
        double bestBackspin = 0;
        double minError = Double.MAX_VALUE;
        
        // Search through possible launch angles
        for (double angle = Constants.CalculateShooter.MIN_ANGLE; angle <= Constants.CalculateShooter.MAX_ANGLE; angle += 1.0) {
            // For each angle, find minimum velocity needed
            double minVelocity = findMinimumVelocityForAngle(angle, distance, targetHeight);
            
            if (minVelocity <= 0 || minVelocity > Constants.CalculateShooter.MAX_LAUNCH_VELOCITY) {
                continue;
            }
            
            // Calculate required backspin for stability and accuracy
            double backspinRPS = calculateRequiredBackspinRPS(minVelocity, angle, distance, targetHeight);
            
            // Simulate trajectory with backspin to verify
            double[] hitPosition = simulateTrajectory(minVelocity, angle, backspinRPS);
            double hitDistance = hitPosition[0];
            double hitHeight = hitPosition[1];
            
            // Calculate error (weighted more heavily for height error)
            double distanceError = Math.abs(hitDistance - distance);
            double heightError = Math.abs(hitHeight - targetHeight);
            double totalError = distanceError + 2.0 * heightError; // Height is more critical
            
            if (totalError < minError) {
                minError = totalError;
                bestAngle = angle;
                bestVelocity = minVelocity;
                bestBackspin = backspinRPS;
            }
        }
        
        // If no solution found with backspin, try without backspin
        if (bestVelocity == 0) {
            return findOptimalTrajectoryNoBackspin(distance, targetHeight);
        }
        
        return new double[]{bestAngle, bestVelocity, bestBackspin};
    }
    
    /**
     * Fallback method - find trajectory without backspin
     */
    private static double[] findOptimalTrajectoryNoBackspin(double distance, double targetHeight) {
        double bestAngle = 45.0;
        double bestVelocity = 0;
        double minError = Double.MAX_VALUE;
        
        for (double angle = Constants.CalculateShooter.MIN_ANGLE; angle <= Constants.CalculateShooter.MAX_ANGLE; angle += 1.0) {
            double minVelocity = findMinimumVelocityForAngle(angle, distance, targetHeight);
            
            if (minVelocity <= 0 || minVelocity > Constants.CalculateShooter.MAX_LAUNCH_VELOCITY) {
                continue;
            }
            
            // Simple trajectory without backspin
            double angleRad = Math.toRadians(angle);
            double timeToTarget = distance / (minVelocity * Math.cos(angleRad));
            double heightAtTarget = minVelocity * Math.sin(angleRad) * timeToTarget - 
                                   0.5 * Constants.CalculateShooter.GRAVITY * timeToTarget * timeToTarget;
            
            double error = Math.abs(heightAtTarget - targetHeight);
            
            if (error < minError) {
                minError = error;
                bestAngle = angle;
                bestVelocity = minVelocity;
            }
        }
        
        return new double[]{bestAngle, bestVelocity, 0.0}; // No backspin
    }
    
    /**
     * Find minimum velocity needed for a given angle to reach target
     */
    private static double findMinimumVelocityForAngle(double angleDeg, double distance, double targetHeight) {
        double angleRad = Math.toRadians(angleDeg);
        
        // Binary search for minimum velocity
        double lowVel = Constants.CalculateShooter.MIN_LAUNCH_VELOCITY;
        double highVel = Constants.CalculateShooter.MAX_LAUNCH_VELOCITY;
        double bestVel = -1;
        
        for (int iter = 0; iter < 20; iter++) {
            double midVel = (lowVel + highVel) / 2;
            
            // Calculate height at target distance (simplified no-drag version)
            double timeToTarget = distance / (midVel * Math.cos(angleRad));
            double heightAtTarget = midVel * Math.sin(angleRad) * timeToTarget - 
                                   0.5 * Constants.CalculateShooter.GRAVITY * timeToTarget * timeToTarget;
            
            if (heightAtTarget >= targetHeight) {
                bestVel = midVel;
                highVel = midVel;
            } else {
                lowVel = midVel;
            }
        }
        
        return bestVel;
    }
    
    /**
     * Calculate required backspin in RPS for stable trajectory
     */
    private static double calculateRequiredBackspinRPS(double velocity, double angleDeg, double distance, double targetHeight) {
        double angleRad = Math.toRadians(angleDeg);
        
        // Backspin creates Magnus effect lift
        // Required lift depends on trajectory and target
        
        // Calculate time of flight approximation
        double flightTime = 2.0 * velocity * Math.sin(angleRad) / Constants.CalculateShooter.GRAVITY;
        
        // Calculate spin rate needed for stabilization
        // Based on ball diameter and velocity (dimensionless spin factor)
        // Typical spin rates for sports balls: 5-50 RPS
        
        // Base spin proportional to velocity and inversely proportional to ball diameter
        double baseSpinRPS = (velocity / Constants.CalculateShooter.BALL_DIAMETER) * 0.15; // Empirical factor
        
        // Adjust for target height - higher targets need more spin for lift
        double heightRatio = targetHeight / distance;
        double heightFactor = 0.8 + heightRatio; // More spin for higher targets
        
        // Adjust for angle - steeper angles need different spin
        double angleFactor = 0.7 + 0.5 * Math.sin(angleRad);
        
        // Calculate required spin
        double requiredSpinRPS = baseSpinRPS * heightFactor * angleFactor;
        
        // Limit to physically reasonable range (max ~80 RPS for 8" ball)
        return Math.min(requiredSpinRPS, 80.0);
    }
    
    /**
     * Simulate trajectory with backspin effects
     */
    private static double[] simulateTrajectory(double velocity, double angleDeg, double backspinRPS) {
        double angleRad = Math.toRadians(angleDeg);
        double vx = velocity * Math.cos(angleRad);
        double vy = velocity * Math.sin(angleRad);
        
        // Convert RPS to angular velocity in radians per second
        double spinRate = backspinRPS * 2.0 * Math.PI;
        
        // Simulation parameters
        double dt = 0.01; // Time step (seconds)
        double maxTime = 5.0; // Maximum simulation time
        double x = 0;
        double y = 0;
        double t = 0;
        
        while (t < maxTime && y >= 0) {
            // Calculate velocity magnitude
            double v = Math.sqrt(vx*vx + vy*vy);
            
            // Drag force (opposite to velocity direction)
            double dragForce = 0.5 * Constants.CalculateShooter.AIR_DENSITY * Constants.CalculateShooter.DRAG_COEFFICIENT * Constants.CalculateShooter.BALL_CROSS_SECTION * v * v;
            double dragAccel = dragForce / Constants.CalculateShooter.BALL_MASS;
            
            // Magnus force due to backspin (perpendicular to velocity)
            // Simplified Magnus effect: F_magnus = CL * ρ * ω × v
            // Lift coefficient increases with spin rate up to a point
            double spinRatio = spinRate * Constants.CalculateShooter.BALL_DIAMETER / v; // Dimensionless spin factor
            double magnusCoeff = Math.min(0.3, 0.1 + 0.5 * spinRatio); // Empirical
            
            double magnusForce = magnusCoeff * Constants.CalculateShooter.AIR_DENSITY * spinRate * v * Constants.CalculateShooter.BALL_CROSS_SECTION;
            double magnusAccel = magnusForce / Constants.CalculateShooter.BALL_MASS;
            
            // Update velocities with forces
            if (v > 0.01) {
                vx -= dragAccel * (vx / v) * dt;
                vy -= dragAccel * (vy / v) * dt;
                vy += magnusAccel * (vx / v) * dt; // Magnus lifts upward
            }
            
            // Gravity
            vy -= Constants.CalculateShooter.GRAVITY * dt;
            
            // Update position
            x += vx * dt;
            y += vy * dt;
            
            t += dt;
        }
        
        return new double[]{x, y};
    }
    
    /**
     * Calculate required wheel RPS from desired ball velocity and backspin
     */
    private static double[] calculateWheelRPS(double requiredBallVelocity, double requiredBallBackspinRPS) {
        // LAUNCHER WHEEL CALCULATIONS
        
        // Calculate required wheel surface speed for desired ball exit velocity
        // Ball velocity is typically ~90% of wheel surface speed due to slippage
        double wheelSurfaceSpeed = requiredBallVelocity / 0.9; // ft/s
        
        // Convert to wheel RPS (revolutions per second)
        double launcherWheelRPS = wheelSurfaceSpeed / Constants.CalculateShooter.LAUNCHER_WHEEL_CIRCUMFERENCE;
        
        // Verify against energy requirements
        // Energy to accelerate ball: 0.5 * m * v²
        double ballKE = 0.5 * Constants.CalculateShooter.BALL_MASS * requiredBallVelocity * requiredBallVelocity;
        
        // Account for flywheel inertia and energy transfer efficiency
        double requiredWheelKE = ballKE / (Constants.CalculateShooter.RESTITUTION_COEFFICIENT * Constants.CalculateShooter.RESTITUTION_COEFFICIENT);
        
        // Calculate required wheel RPS from kinetic energy
        // KE_rotational = 0.5 * I * (2π * RPS)²
        double requiredOmega = Math.sqrt(2.0 * requiredWheelKE / Constants.CalculateShooter.LAUNCHER_WHEEL_MOMENT_OF_INERTIA);
        double requiredWheelRPSFromEnergy = requiredOmega / (2.0 * Math.PI);
        
        // Use the higher of the two RPS requirements to ensure enough energy
        double targetLauncherRPS = Math.max(launcherWheelRPS, requiredWheelRPSFromEnergy);
        
        // BACKSPIN WHEEL CALCULATIONS
        
        // Required backspin wheel RPS (ball spin is less than wheel spin)
        // Backspin wheels need to spin faster to impart the required ball spin
        double backspinWheelRPS = requiredBallBackspinRPS / Constants.CalculateShooter.BACKSPIN_TRANSFER_RATIO;
        
        // Account for multiple wheels and their inertia
        // Verify energy requirements for backspin
        double requiredBackspinOmega = backspinWheelRPS * 2.0 * Math.PI;
        double backspinKE = 0.5 * Constants.CalculateShooter.BACKSPIN_TOTAL_INERTIA * requiredBackspinOmega * requiredBackspinOmega;
        
        // Add a small factor for maintaining spin under load
        if (backspinWheelRPS > 0) {
            double spinLoadFactor = 1.0 + (Math.abs(requiredBallBackspinRPS) / 80.0) * 0.2;
            backspinWheelRPS *= spinLoadFactor;
        }
        
        // Apply mechanical geometry correction
        // Backspin motor positioned behind and above affects contact efficiency
        double geometryFactor = Math.cos(Constants.CalculateShooter.CONTACT_POINT_ANGLE) * 0.8 + 0.4;
        backspinWheelRPS /= geometryFactor;
        
        // Clamp values to realistic ranges based on motor capabilities
        targetLauncherRPS = clamp(targetLauncherRPS, 5.0, Constants.CalculateShooter.LAUNCHER_MAX_RPS * 0.95);
        backspinWheelRPS = clamp(backspinWheelRPS, 2.0, Constants.CalculateShooter.BACKSPIN_MAX_RPS * 0.95);
        
        // Backspin motor runs in reverse for backspin (negative RPS)
        return new double[]{targetLauncherRPS, -backspinWheelRPS};
    }
    
    /**
     * Get recommended shot for quick lookup - returns RPS values
     */
    public static double[] getRecommendedShot(double distance, double height) {
        return calculateOptimalShot(distance, height);
    }
    
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    
    /**
     * Convert RPM to RPS
     */
    public static double rpmToRps(double rpm) {
        return rpm / 60.0;
    }
}
