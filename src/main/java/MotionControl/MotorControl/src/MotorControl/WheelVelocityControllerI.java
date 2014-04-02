package MotorControl;

/**
 * <p>
 * Closed-loop integral wheel velocity controller.
 * </p>
 * 
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerI extends WheelVelocityController {

    /**
     * <p>
     * Integrator maximum value, to prevent over controlling.
     * </p>
     */
    protected static final double INTEGRATOR_MAX = 200.0;

    /**
     * <p>
     * Integrator minimum value.
     * </p>
     */
    protected static final double INTEGRATOR_MIN = -200.0;

    /**
     * <p>
     * The result of the previous control step.
     * </p>
     **/
    protected double lastResult = 0;

    /**
     * <p>
     * Integrator for angular velocity error.
     * </p>
     */
    protected double errorIntegrator = 0;
    
    /**
     * Special case variable to stop completely on 0 velocity.
     */
    protected boolean stopped = false;

    /**
     * {@inheritDoc}
     * 
     * <p>
     * This impl implements closed-loop integral control.
     * </p>
     **/
    public double controlStep() {

        double result = 0;

        // Start Student Code
        // Update and clamp the integrator
        errorIntegrator += (computeAngularVelocity() - getDesiredAngularVelocity())
                * sampleTime;
        if (errorIntegrator > INTEGRATOR_MAX)
            errorIntegrator = INTEGRATOR_MAX;
        if (errorIntegrator < INTEGRATOR_MIN)
            errorIntegrator = INTEGRATOR_MIN;
        if (!stopped) {
            result = -gain * errorIntegrator + -10
                    * (computeAngularVelocity() - getDesiredAngularVelocity())
                    + PWM_PER_ANGVEL * getDesiredAngularVelocity();
        }
        else {
            result = 0;
        }
        // System.out.println("Integrator error: " + errorIntegrator);
        // System.out.println("Feed forward: " + getDesiredAngularVelocity());
        // End Student Code

        if (result > MAX_PWM)
            result = MAX_PWM;

        if (result < -MAX_PWM)
            result = -MAX_PWM;

        lastResult = result;

	System.out.println("WheelVelocityController: "
			   + getDesiredAngularVelocity() + ", "
			   + result);

        return result;
    }
    
    @Override
    public void setDesiredAngularVelocity(double vel) {
	final double STOPPED_VEL_THRESH = 0.05;
        // Set stopped flag on 0 vel command
        if (Math.abs(vel) < STOPPED_VEL_THRESH) {
	    errorIntegrator = 0;
            stopped = true;
	}
        else {
            stopped = false;
	}
        // Reset the integrator on every change
        super.setDesiredAngularVelocity(vel);
    }

    /**
     * {@inheritDoc}
     * 
     * <p>
     * This impl returns "I".
     * </p>
     **/
    public String getName() {
        return "I";
    }
}
