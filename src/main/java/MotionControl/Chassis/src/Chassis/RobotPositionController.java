package Chassis;

//import MotorControlSolution.*;
import MotorControl.*;

/**
 * <p>
 * A whole-robot position controller.
 * </p>
 **/
public class RobotPositionController {

    /**
     * <p>
     * Student Code: radius of the wheel on the motor (in meters). This should
     * be a protected static final double called WHEEL_RADIUS_IN_M.
     * </p>
     **/
    protected static final double WHEEL_RADIUS_IN_M = .0625;

    /**
     * <p>
     * Student Code: encoder ticks per motor revolution.
     * </p>
     * This should be a protected static final double called
     * TICKS_PER_REVOLUTION.</p> Hint: TICKS_PER_REVOLUTION units:
     * ticks/revolution (WITH gear ratio term)
     **/
    protected static final double TICKS_PER_REVOLUTION = 131000;

    /**
     * <p>
     * Constant to convert linear translational velocity to angular velocity
     * (m/rad)
     */
    protected static final double LINEAR_TO_ANGULAR = WHEEL_RADIUS_IN_M;

    /**
     * <p>
     * Constant to convert ticks to linear translation (ticks/m)
     */
    protected static final double TICKS_TO_M = WheelVelocityController
            .computeRadiansPerTick() * LINEAR_TO_ANGULAR;
    

    /**
     * <p>
     * Constant speed increment for every 100 steps in translate/rotate
     * </p>
     */
    protected static final double SPEED_INC = .1;

    /**
     * <p>
     * The whole-robot velocity controller.
     * </p>
     **/
    protected RobotVelocityController robotVelocityController;

    /**
     * <p>
     * Total ticks since reset, positive means corresp side of robot moved
     * forward.
     * </p>
     **/
    protected volatile double[] totalTicks = new double[2];

    /**
     * <p>
     * Total elapsed time since reset in seconds.
     * </p>
     **/
    protected double totalTime = 0.0;

    /**
     * <p>
     * Time in seconds since last update.
     * </p>
     **/
    protected double sampleTime;

    /**
     * <p>
     * An abstract gain; meaning depends on the particular subclass
     * implementation.
     * </p>
     **/
    protected double gain = 1.0;

    /**
     * <p>
     * The robot.
     * </p>
     **/
    protected OdometryRobot robot;

    /**
     * <p>
     * Create a new position controller for a robot.
     * </p>
     * 
     * @param robot
     *            the robot, not null
     **/
    public RobotPositionController(OdometryRobot robot) {
        this.robot = robot;
    }

    /**
     * <p>
     * Translate at the specified speed for the specified distance.
     * </p>
     * 
     * <p>
     * Blocks until the motion is complete or errored.
     * </p>
     * 
     * @param speed
     *            the desired robot motion speed in m/s
     * @param distance
     *            the desired distance to move in meters, relative to the
     *            robot's pose at time of call.
     * 
     * @return true iff motion was successful
     **/

    public boolean translate(double speed, double distance) {
        boolean ok = true;
        // Begin Student Code
        setRobotVelocityController(robot.getRobotVelocityController());
        double ticksToMeters = 1.0 / TICKS_PER_REVOLUTION * WHEEL_RADIUS_IN_M
                * 2.0 * Math.PI;
        double prevticks = (totalTicks[RobotBase.LEFT] + totalTicks[RobotBase.RIGHT]) / 2;
        double currticks = (totalTicks[RobotBase.LEFT] + totalTicks[RobotBase.RIGHT]) / 2;
        double currdist = 0;
        double setspeed = 0;
        double currspeed = 0;
        int numtries = 0;
        int iter = 0;
        double gain = 20;

        int direction = 1;
        if (distance < 0) {
            direction = -1;
            distance = distance * -1;
        }

        while (currdist < distance - .01 && ok) {
            // we want our speed to be proportional to the distance remaining so
            // that we slow down
            // as we approach the goal, without ever going faster than our
            // setpoint speed
            setspeed = Math.min(speed, (distance - currdist) * gain);

            if (currspeed < setspeed) {
                if (iter % 100 == 0)
                    currspeed += SPEED_INC; // limit acceleration when starting up
            }
            if (currspeed > setspeed) {
                currspeed = setspeed;
            }

            iter++;

            // send output to wheels
            robotVelocityController.setDesiredAngularVelocity(currspeed
                    * direction, currspeed * direction);

            // debugging
            System.out.println("\tSetpoint speed: " + currspeed);
            System.out.println("\tDistance: " + currdist);

            // update current encoder value
            currticks = (totalTicks[RobotBase.LEFT] + totalTicks[RobotBase.RIGHT]) / 2;

            // check for error/timeout
            if (iter % 1000 == 0) {
                if (currticks - prevticks < 10) // check for failure to move
                    numtries++;
                if (numtries == 100) // time out
                    ok = false;
            }

            // calculate distance travelled
            currdist += (currticks - prevticks) * ticksToMeters * direction;
            prevticks = currticks;
        }

        // make sure the robot is stopped
        robotVelocityController.setDesiredAngularVelocity(0, 0);

        // End Student Code
        return ok;
    }

    /**
     * <p>
     * Rotate at the specified speed for the specified angle.
     * </p>
     * 
     * <p>
     * Blocks until the motion is complete or errored.
     * </p>
     * 
     * 
     * @param speed
     *            the desired robot motion speed in radians/s
     * @param angle
     *            the desired angle to rotate in radians, relative to the
     *            robot's pose at time of call.
     * 
     * @return true iff motion was successful
     **/
    public boolean rotate(double speed, double angle) {
        
        boolean ok = true;
        // negative angle is cw, left forward, right backward
        // positive angle is ccw, left backward, right forward
        double[] currticks = new double[2];
        double[] prevticks = new double[2];
        double[] setspeed = new double[2];
        for (int i = 0; i < 2; i++) {
            currticks[i] = totalTicks[i];
            prevticks[i] = totalTicks[i];
        }

        double ticksToMeters = 1.0 / TICKS_PER_REVOLUTION * WHEEL_RADIUS_IN_M
                * 2.0 * Math.PI;
        double angleToDistance = RobotBase.WHEELBASE / 2;

        double[] currdist = new double[2];
        double[] currspeed = new double[2];
        double[] distance = new double[2];
        int numtries = 0;
        int iter = 0;
        double[] dir = new double[2];
        int gain = 10;

        if (angle < 0) {
            dir[RobotBase.LEFT] = 1;
            dir[RobotBase.RIGHT] = -1;
        }
        else {
            dir[RobotBase.LEFT] = -1;
            dir[RobotBase.RIGHT] = 1;
        }

        distance[RobotBase.LEFT] = Math.abs(angle) * angleToDistance
                * dir[RobotBase.LEFT];
        distance[RobotBase.RIGHT] = Math.abs(angle) * angleToDistance
                * dir[RobotBase.RIGHT];

	boolean rightDone = false;
	boolean leftDone = false;

        while (!rightDone && !leftDone && ok){

            for (int i = 0; i < 2; i++) {
                // we want our speed to be proportional to the distance
                // remaining so that we slow down
                // as we approach the goal, without ever going faster than our
                // setpoint speed
                setspeed[i] = dir[i]*Math.min(speed, (Math.abs(distance[i] - currdist[i]))
                        * gain);

                if (Math.abs(currspeed[i]) < Math.abs(setspeed[i])) {
                    if (iter % 100 == 0)
                        currspeed[i] = currspeed[i] + dir[i]*SPEED_INC; // limit acceleration when starting
                                            // up
                }
                if (Math.abs(currspeed[i]) > Math.abs(setspeed[i])) {
                    currspeed[i] = setspeed[i];
                }
            }
            iter++;

            // send output to wheels
            // TODO: set offset for controller
            robotVelocityController.setDesiredAngularVelocity(
                    currspeed[RobotBase.LEFT], currspeed[RobotBase.RIGHT]);

            // debugging prints
            System.out.println("\tSetpoint speedL: "
                    + currspeed[RobotBase.LEFT]);
            System.out.println("\tSetpoint speedR: "
                    + currspeed[RobotBase.RIGHT]);
            System.out.println("\tDistanceL: " + currdist[RobotBase.LEFT] + " of " + distance[RobotBase.LEFT]);
            System.out.println("\tDistanceR: " + currdist[RobotBase.RIGHT] + "of " + distance[RobotBase.RIGHT]);

            for (int i = 0; i < 2; i++) {
                currticks[i] = totalTicks[i];
            }
            // System.out.println("\tCurrTicksL: " + currticks[RobotBase.LEFT]);
            // System.out.println("\tCurrTicksR: " +
            // currticks[RobotBase.RIGHT]);

            // error checking & timeout
            if (iter % 1000 == 0) {
                for (int i = 0; i < 2; i++) {
                    if (Math.abs( currticks[i] - prevticks[i]) < 10) // check for failure
                                                          // to move
                        numtries++;
                    if (numtries == 100) // time out
                        ok = false;
                }
            }

            // calculate distance travelled
            for (int i = 0; i < 2; i++) {
                double diff = currticks[i] - prevticks[i];
                    // System.out.println("\thiiiiiiiiiiiiiiiii " +
                    // (prevticks[i] - currticks[i]));
                currdist[i] = currdist[i] + diff*ticksToMeters;
                prevticks[i] = currticks[i];
            }

	    if( Math.abs(currdist[RobotBase.RIGHT] - distance[RobotBase.RIGHT]) < .01)
		rightDone = true;
	    if( Math.abs(currdist[RobotBase.LEFT] - distance[RobotBase.LEFT]) < .01)
		leftDone = true;
        }
        // Begin Student Code
	
        // End Student Code
        robotVelocityController.setDesiredAngularVelocity(0, 0);


        return ok;
    }

    /**
     * <p>
     * If position control is closed-loop, this computes the new left and right
     * velocity commands and issues them to {@link #robotVelocityController}.
     * </p>
     **/
    public synchronized void controlStep() {

        if (robotVelocityController == null)
            return;

        if (!robot.motorsEnabled() || robot.estopped())
            return;

        // Begin Student Code (if implementing closed-loop control)
        // End Student Code (if implementing closed-loop control)
    }

    /**
     * <p>
     * Set the whole-robot velocity controller.
     * </p>
     * 
     * <p>
     * This is called automatically by {@link OdometeryRobot}.
     * </p>
     * 
     * @param vc
     *            the whole-robot velocity controller
     **/
    public void setRobotVelocityController(RobotVelocityController vc) {
        robotVelocityController = vc;
    }

    /**
     * <p>
     * Set {@link #gain}.
     * </p>
     * 
     * @param g
     *            the new gain
     **/
    public void setGain(double g) {
        gain = g;
    }

    /**
     * <p>
     * Get {@link #gain}.
     * </p>
     * 
     * @return gain
     **/
    public double getGain() {
        return gain;
    }

    /**
     * <p>
     * Update feedback and sample time.
     * </p>
     * 
     * @param time
     *            the time in seconds since the last update, saved to
     *            {@link #sampleTime}
     * @param leftTicks
     *            left encoder ticks since last update, positive means corresp
     *            side of robot rolled forward
     * @param rightTicks
     *            right encoder ticks since last update, positive means corresp
     *            side of robot robot rolled forward
     **/
    public synchronized void update(double time, double leftTicks,
            double rightTicks) {

        sampleTime = time;

        totalTicks[RobotBase.LEFT] += leftTicks;
        totalTicks[RobotBase.RIGHT] += rightTicks;
        totalTime += time;
    }
}
