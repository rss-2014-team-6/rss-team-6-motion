package MotorControl;

/**
 * <p>
 * Feed-forward wheel velocity controller.
 * </p>
 * 
 * @author vona
 * @author prentice
 **/
public class WheelVelocityControllerFF extends WheelVelocityController {

    /**
     * {@inheritDoc}
     * 
     * <p>
     * This impl implements simple feed-forward control.
     * </p>
     **/
    public double controlStep() {
        // Motion Control Lab, Part 7
        return gain * getDesiredAngularVelocity() * PWM_PER_ANGVEL;
	
    }

    /**
     * {@inheritDoc}
     * 
     * <p>
     * This impl returns "FF".
     * </p>
     **/
    public String getName() {
        return "FF";
    }
}
