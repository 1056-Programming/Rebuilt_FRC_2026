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
        double requiredBackspinRPM = optimalParams[2];
        
        // Convert to motor powers
        double[] motorPowers = calculateMotorPowers(requiredVelocity, requiredBackspinRPM);
        
        return new double[]{
            motorPowers[0]/100,          // Launcher power %
            motorPowers[1]/100,          // Backspin power %
            optimalAngle,             // Launch angle
            requiredVelocity,         // Exit velocity (ft/s)
            requiredBackspinRPM       // Backspin RPM
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
            double backspinRPM = calculateRequiredBackspin(minVelocity, angle, distance, targetHeight);
            
            // Simulate trajectory with backspin to verify
            double[] hitPosition = simulateTrajectory(minVelocity, angle, backspinRPM);
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
                bestBackspin = backspinRPM;
            }
        }
        
        return new double[]{bestAngle, bestVelocity, bestBackspin};
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
     * Calculate required backspin for stable trajectory
     */
    private static double calculateRequiredBackspin(double velocity, double angleDeg, double distance, double targetHeight) {
        double angleRad = Math.toRadians(angleDeg);
        
        // Backspin creates Magnus effect lift
        // Required lift depends on trajectory and target
        
        // Calculate time of flight
        double flightTime = 2.0 * velocity * Math.sin(angleRad) / Constants.CalculateShooter.GRAVITY;
        
        // Calculate spin rate needed for stabilization
        // Higher velocities and longer distances need more spin
        double baseSpin = (velocity / 10.0) * 500; // RPM scaling
        
        // Adjust for target height
        double heightRatio = targetHeight / distance;
        double heightFactor = 0.5 + heightRatio; // More spin for higher targets
        
        // Adjust for angle
        double angleFactor = Math.sin(angleRad) * 1.5;
        
        // Calculate required spin
        double requiredSpin = baseSpin * heightFactor * angleFactor;
        
        // Limit to achievable range
        return Math.min(requiredSpin, Constants.CalculateShooter.BACKSPIN_MAX_RPM * 0.9);
    }
    
    /**
     * Simulate trajectory with backspin effects
     */
    private static double[] simulateTrajectory(double velocity, double angleDeg, double backspinRPM) {
        double angleRad = Math.toRadians(angleDeg);
        double vx = velocity * Math.cos(angleRad);
        double vy = velocity * Math.sin(angleRad);
        
        // Convert RPM to radians per second
        double spinRate = (backspinRPM * 2.0 * Math.PI) / 60.0;
        
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
            double magnusCoeff = 0.2; // Lift coefficient approximation
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
     * Calculate motor powers from required velocity and backspin
     */
    private static double[] calculateMotorPowers(double requiredVelocity, double requiredBackspinRPM) {
        // LAUNCHER MOTOR CALCULATIONS
        
        // Calculate required wheel RPM for desired ball exit velocity
        // Ball velocity is typically ~90% of wheel surface speed due to slippage
        double wheelSurfaceSpeed = requiredVelocity / 0.9; // ft/s
        double launcherWheelRPM = (wheelSurfaceSpeed / Constants.CalculateShooter.LAUNCHER_WHEEL_CIRCUMFERENCE) * 60.0;
        
        // Calculate power needed based on kinetic energy transfer
        // Energy to accelerate ball: 0.5 * m * v²
        double ballKE = 0.5 * Constants.CalculateShooter.BALL_MASS * requiredVelocity * requiredVelocity;
        
        // Account for flywheel inertia and energy transfer efficiency
        double requiredWheelKE = ballKE / (Constants.CalculateShooter.RESTITUTION_COEFFICIENT * Constants.CalculateShooter.RESTITUTION_COEFFICIENT);
        
        // Calculate required wheel speed from kinetic energy
        // KE_rotational = 0.5 * I * ω²
        double requiredOmega = Math.sqrt(2.0 * requiredWheelKE / Constants.CalculateShooter.LAUNCHER_WHEEL_MOMENT_OF_INERTIA);
        double requiredWheelRPMFromEnergy = (requiredOmega * 60.0) / (2.0 * Math.PI);
        
        // Use the higher of the two RPM requirements
        double targetLauncherRPM = Math.max(launcherWheelRPM, requiredWheelRPMFromEnergy);
        
        // Calculate launcher power percentage (simplified motor model)
        // Power ∝ RPM² for given torque load
        double launcherPowerPercent = (targetLauncherRPM / Constants.CalculateShooter.LAUNCHER_MAX_RPM) * 100.0;
        launcherPowerPercent = Math.pow(launcherPowerPercent / 100.0, 0.5) * 100.0; // Square root scaling
        
        // BACKSPIN MOTOR CALCULATIONS
        
        // Required backspin wheel RPM (ball spin is less than wheel spin)
        double backspinWheelRPM = requiredBackspinRPM / Constants.CalculateShooter.BACKSPIN_TRANSFER_RATIO;
        
        // Calculate backspin power needed
        // Account for multiple wheels and their inertia
        double requiredBackspinOmega = (backspinWheelRPM * 2.0 * Math.PI) / 60.0;
        double backspinKE = 0.5 * Constants.CalculateShooter.BACKSPIN_TOTAL_INERTIA * requiredBackspinOmega * requiredBackspinOmega;
        
        // Additional power for maintaining spin under load
        double backspinPowerFactor = 1.0 + (Math.abs(requiredBackspinRPM) / Constants.CalculateShooter.BACKSPIN_MAX_RPM) * 0.3;
        
        // Calculate backspin power percentage
        double backspinPowerPercent = (backspinWheelRPM / Constants.CalculateShooter.BACKSPIN_MAX_RPM) * 100.0 * backspinPowerFactor;
        
        // Apply mechanical geometry correction
        // Backspin motor positioned behind and above affects contact efficiency
        double geometryFactor = Math.cos(Constants.CalculateShooter.CONTACT_POINT_ANGLE) * 0.8 + 0.4;
        backspinPowerPercent /= geometryFactor;
        
        // Clamp values to realistic ranges
        launcherPowerPercent = clamp(launcherPowerPercent, 15.0, 95.0);
        backspinPowerPercent = clamp(backspinPowerPercent, 10.0, 95.0);
        
        // Backspin motor runs in reverse for backspin
        return new double[]{launcherPowerPercent, -backspinPowerPercent};
    }
    
    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    

}
