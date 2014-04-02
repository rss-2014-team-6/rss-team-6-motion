package motion.Motor;

import MotorControl.RobotBase;
import MotorControl.RobotVelocityController;
import MotorControl.WheelVelocityController;

public class RobotPositionController extends RobotVelocityController {

    // overall strategy: rotate to face point, drive straight, adjust theta if necessary

    protected double x;
    protected double y;
    protected double theta;
    protected double xGoal;
    protected double yGoal;
    protected double thetaGoal;
    protected static final double maxSpeed = 4.0;
    protected static final double minSpeed = 2.0;
    protected static final double rotationGain = 4.0; // Rotational gain for pure rotation to point
    protected static final double arcRotationGain = 1.0; // Rotational gain for arcing rotation component
    protected static final double translationGain = 7.0; // Translational gain for arcing/linear motion

    // ranges are in meters and bearings are in radians
    protected static final double RANGE_THRESHOLD = 0.05;
    protected static final double TRANSLATION_TO_ANGVEL = 1.0 / RobotBase.WHEEL_RADIUS_IN_M;
    protected static final double RANGE_KP = 1.0;
    protected static final double BEARING_THRESHOLD = 0.25;
    protected static final double ROTATION_TO_ANGVEL = (RobotBase.WHEELBASE / 2.0) * TRANSLATION_TO_ANGVEL;
    protected static final double BEARING_KP = 2.0;
    protected static final double THRESHOLD = .05;
    protected static final double THETA_THRESHOLD = .1;

    protected int direction;

    public RobotPositionController(
            WheelVelocityController leftWheelVelocityController,
            WheelVelocityController rightWheelVelocityController,
            double x, double y, double theta) {
        super(leftWheelVelocityController, rightWheelVelocityController);
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.xGoal = x;
        this.yGoal = y;
        this.thetaGoal = theta;
	this.direction = 1;
	//System.out.println("I exist :O");
    }

    public void setReverse(boolean rev){
	if(rev)
	    direction = -1;
	else
	    direction = 1;
    }

    /**
     *  Normalize a given theta to the range -pi to pi
     */
    protected static double normalizeTheta(double theta) {
	// Normalize to either [0, 2*pi) or (-2*pi, 0], depending on sign
	theta = theta % (Math.PI*2);
	if (theta < -Math.PI) {
	    theta += Math.PI*2;
	}
	else if (theta > Math.PI) {
	    theta -= Math.PI*2;
	}
	return theta;
    }

    public void setPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
	//controlStep(new double[2]);
	//System.out.println("pose set");
    }

    public void setGoal(double x, double y, double theta) {
        this.xGoal = x;
        this.yGoal = y;
        this.thetaGoal = theta; // theta goal of -1 means no theta goal
    	//System.out.println("goal set");
    }

    public void controlStep(double[] control) {
        double xError = x - xGoal;
        double yError = y - yGoal;
        // double thetaError = theta - thetaGoal;
       	System.out.println("");	
	System.out.println("control step x: " + x + "     y: " + y + "     theta: " + theta);
	System.out.println("goal x: " +xGoal+ "     y: " + yGoal + "     thetaGoal: " + thetaGoal);
	System.out.println("");

        if (theta == -1)
            System.out.println("Error: need to have odometry data before moving.");

        if (Math.abs(xError) < THRESHOLD && Math.abs(yError) < THRESHOLD) {
            // we've reached the target point, if necessary rotate to theta

            if (thetaGoal != -1) {
                // we need to rotate

		// Get theta error in the range -pi to pi
                double thetaError = normalizeTheta(thetaGoal - theta);

                if (Math.abs(thetaError) > THETA_THRESHOLD) {
                    // we have a goal and we're not there

                    double wheelAngVel = Math.min(maxSpeed, rotationGain * Math.abs(thetaError));
                    wheelAngVel = Math.max(minSpeed, wheelAngVel);
                    if (thetaError < 0)
                        wheelAngVel *= -1; // is this the right direction?
                    System.out.println("turning to adjust final theta");
                    System.out.println("angvel: " + wheelAngVel);

                    wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(-wheelAngVel);
                    wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(wheelAngVel);
                }
                else {
                    // we've reached the goal!
                    wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(0);
                    wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(0);
                }
            }
            else {
                // we've reached the goal!
                wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(0);
                wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(0);
		System.out.println("Robot thinks it reached the goal");
            }
        }
        else {
            // we need to head towards the target point
            // this should be range -pi to pi
            double theta_to_point = Math.atan2(yGoal - y, xGoal - x);
	    double thetaError = normalizeTheta(theta_to_point - theta);

	    if(direction < 0){ // reverse
		thetaError = normalizeTheta(thetaError + Math.PI);
	    }

            if (Math.abs(thetaError) < THETA_THRESHOLD) {
                // we are already headed towards the appropriate point

                double distError = Math.sqrt(Math.pow(xGoal - x, 2) + Math.pow(yGoal - y, 2));

                double wheelAngVel = Math.min(maxSpeed, translationGain * Math.sqrt(Math.abs(distError)));
                wheelAngVel = Math.max(minSpeed, wheelAngVel);
		double wheelDiffVel = Math.min(maxSpeed, arcRotationGain * Math.abs(thetaError));
		if (thetaError < 0) wheelDiffVel *= -1;

                System.out.println("forwarding :P");
                System.out.println("angvel: " + wheelAngVel);
		System.out.println("diffvel: " + wheelDiffVel);
		System.out.println("theta error: " + thetaError);
		
                wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(direction * (wheelAngVel - wheelDiffVel));
                wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(direction * (wheelAngVel + wheelDiffVel));
            }
            else {
                // purely rotate to face that point

                double wheelAngVel = Math.min(maxSpeed, rotationGain * Math.sqrt(Math.abs(thetaError)));
                wheelAngVel = Math.max(minSpeed, wheelAngVel);
                if (thetaError < 0)
                    wheelAngVel *= -1; // is this the right direction?
                System.out.println("turning to face point");
                System.out.println("angvel: " + wheelAngVel);
                wheelVelocityController[RobotBase.LEFT].setDesiredAngularVelocity(-wheelAngVel);
                wheelVelocityController[RobotBase.RIGHT].setDesiredAngularVelocity(wheelAngVel);
            }
        }
	
        super.controlStep(control);
    }

}
