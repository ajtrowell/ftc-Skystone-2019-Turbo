package org.firstinspires.ftc.teamcode.Utilities;
import org.firstinspires.ftc.teamcode.Utilities.MecanumNavigation.Navigation2D;


/**
 * Provide limit to acceleration for mecanum drive control.
 * Units of inputs, outputs, and limits can be anything the user decides, as long as
 * they are consistent.  Velocities are assumed to be distance per second, and acceleration is
 * assumed to be distance per second per second.  Distance could be inches, encoder ticks, or
 * a magnitude scaled from -1.0 to 1.0.
 * Acceleration limits are combined for forward/lateral strafing, but calculated seperately
 * for rotation.
 *
 */
public class AccelerationLimiter {


    boolean limitsSet = false;
    boolean initialized = false;
    public boolean limiterEnabled = true;

    double previousTime = 0;
//    double currentTime = 0;

    Navigation2D previousVelocityAndRotationOutput = new Navigation2D(0,0,0);
    Navigation2D requestedVelocityAndRotation = new Navigation2D(0,0,0);
    Navigation2D newVelocityAndRotationOutput = new Navigation2D(0,0,0);


    // Limit to the change per second allowed for Linear and Rotational Velocity
    public double linearAccelerationLimit;
    public double rotationalAccelerationLimit;

    // Constructor
    public AccelerationLimiter() {
    }

    public AccelerationLimiter(double linearAccelerationLimit, double rotationalAccelerationLimit) {
        setAccelerationLimitsLinearAndRotation(linearAccelerationLimit,rotationalAccelerationLimit);
    }

    public void setAccelerationLimitsLinearAndRotation(double linearAccelerationLimit, double rotationalAccelerationLimit) {
        this.linearAccelerationLimit = Math.abs(linearAccelerationLimit);
        this.rotationalAccelerationLimit = Math.abs(rotationalAccelerationLimit);
        limitsSet = true;
    }

    /**
     * Expected convention is Vx forward, Vy left, Av counter clockwise.
     * However, this convention doesn't matter, as long as you are consistent with the input/output.
     *
     * @param currentTime
     * @param vx
     * @param vy
     * @param av
     */
    public void update(double currentTime, double vx, double vy, double av) {
        if(!limitsSet) {
            throw new RuntimeException("Must run setAccelerationLimitsLinearAndRotation() before update");
        }

        if(initialized) {
            double deltaTime = currentTime - previousTime;
            if (deltaTime == 0.0) return;
            requestedVelocityAndRotation = new Navigation2D(vx,vy,av);
            Navigation2D differenceVector = requestedVelocityAndRotation.subtractAndReturn(previousVelocityAndRotationOutput);

            double linearAccelerationRequested = differenceVector.getMagnitude() / deltaTime;
            double rotationalAccelerationRequested = differenceVector.theta / deltaTime;
            boolean isLinearAccelerationWithinLimit = Math.abs(linearAccelerationRequested) <= linearAccelerationLimit;
            boolean isRotationalAccelerationWithinLimit = Math.abs(rotationalAccelerationRequested) <= rotationalAccelerationLimit;

            // Baseline
            newVelocityAndRotationOutput = requestedVelocityAndRotation.copy();


            if(limiterEnabled && (!isLinearAccelerationWithinLimit || !isRotationalAccelerationWithinLimit)) {
                // Cap linear accelerations to limit.
                if (!isLinearAccelerationWithinLimit) {
                    double magnitudeRescale = (linearAccelerationLimit / linearAccelerationRequested);
                    differenceVector = differenceVector.multiplyAndReturn(magnitudeRescale);
                }
                // Cap rotational acceleration to limit.
                if (!isRotationalAccelerationWithinLimit) {
                    double rotationRescale = (rotationalAccelerationLimit / rotationalAccelerationRequested);
                    differenceVector.theta = differenceVector.theta * rotationRescale;
                }
                // Apply modified difference vector to create new vector.
                newVelocityAndRotationOutput = previousVelocityAndRotationOutput.addAndReturn(differenceVector);
            }
            previousVelocityAndRotationOutput = newVelocityAndRotationOutput; // Update Previous output from new.
        }

        // This will initialize the object if it isn't already.
        previousTime = currentTime;
        initialized = true;



    }

    public Navigation2D getNewVelocityAndRotationOutput() {
        return newVelocityAndRotationOutput;
    }


    public Mecanum.Command updateAndReturnMecanumCommand(double currentTime, Mecanum.Command command) {
        update(currentTime, command.vx, command.vy, command.av);
        return new Mecanum.Command(newVelocityAndRotationOutput.x, newVelocityAndRotationOutput.y, newVelocityAndRotationOutput.theta);
    }




}
