package MotorControl;

/**
 * <p>
 * Open loop wheel velocity controller.
 * </p>
 * 
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerPWM extends WheelVelocityController {

    /**
     * {@inheritDoc}
     * 
     * <p>
     * This impl implements simple open-loop control.
     * </p>
     **/
    public double controlStep() {
        return desiredAngularVelocity;
    }

    /**
     * {@inheritDoc}
     * 
     * <p>
     * This impl returns "PWM".
     * </p>
     **/
    public String getName() {
        return "PWM";
    }
}
