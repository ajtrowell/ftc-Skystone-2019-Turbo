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

    Navigation2D previousVelocityAndRotation;
    Navigation2D requestedVelocityAndRotation;
    Navigation2D newVelocityAndRotation;


    // Limit to the change per second allowed for Linear and Rotational Velocity
    double linearAccelerationLimit;
    double rotationalAccelerationLimit;

    // Constructor
    AccelerationLimiter() {
    }

    AccelerationLimiter(double linearAccelerationLimit, double rotationalAccelerationLimit) {
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
            requestedVelocityAndRotation = new Navigation2D(vx,vy,av);
            Navigation2D differenceVector = requestedVelocityAndRotation.subtractAndReturn(previousVelocityAndRotation);

            double linearAccelerationRequested = differenceVector.getMagnitude() * deltaTime;
            double rotationalAccelerationRequested = differenceVector.theta * deltaTime;
            boolean isLinearAccelerationWithinLimit = Math.abs(linearAccelerationRequested) <= linearAccelerationLimit;
            boolean isRotationalAccelerationWithinLimit = Math.abs(rotationalAccelerationRequested) <= rotationalAccelerationLimit;

            // Baseline
            newVelocityAndRotation = requestedVelocityAndRotation.copy();

            if(limiterEnabled) {
                // Cap linear accelerations to limit.
                if (!isLinearAccelerationWithinLimit) {
                    double magnitudeRescale = (linearAccelerationLimit / linearAccelerationRequested);
                    newVelocityAndRotation = newVelocityAndRotation.multiplyAndReturn(magnitudeRescale);
                }
                // Cap rotational acceleration to limit.
                if (!isRotationalAccelerationWithinLimit) {
                    double rotationRescale = (rotationalAccelerationLimit / rotationalAccelerationRequested);
                    newVelocityAndRotation.theta = newVelocityAndRotation.theta * rotationRescale;
                }
            }
        }

        if(!initialized) {
            previousTime = currentTime;
            previousVelocityAndRotation = new Navigation2D(vx,vy,av);
            initialized = true;
        }



    }

    public Navigation2D getNewVelocityAndRotation() {
        return newVelocityAndRotation;
    }

    public Navigation2D updateAndReturnNewVelocityAndRotation(double currentTime, double vx, double vy, double av) {
        update(currentTime,vx,vy,av);
        return getNewVelocityAndRotation();
    }




}
