package motion;

import orc.Orc;

import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.NodeConfiguration;
import org.ros.node.Node;
import org.ros.node.topic.Publisher;
import rss_msgs.ArmMsg;
import rss_msgs.ReverseMsg;
import rss_msgs.MotionMsg;
import rss_msgs.OdometryMsg;
import rss_msgs.PositionTargetMsg;
import rss_msgs.PositionMsg;
import rss_msgs.WaypointMsg;
import rss_msgs.VelocityMsg;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.topic.Subscriber;
import org.apache.commons.logging.Log;
import org.ros.internal.node.topic.PublisherIdentifier;
import org.ros.internal.node.topic.DefaultSubscriber;

import motion.Motor.MotorListenerForPositionControl;
import motion.Motor.MotorListenerForOdometry;
import motion.Motor.MotorListenerForReverse;
import motion.Motor.MotorListenerForWaypoint;
import motion.Motor.MotorListenerForPosition;
import motion.Motor.MotorListenerForVelocity;
import motion.Motor.RobotPositionController;
import motion.Servo.ServoListener;
import MotorControl.RobotBase;
import MotorControl.RobotVelocityController;
import MotorControl.RobotVelocityControllerBalanced;
import MotorControl.WheelVelocityControllerFF;
import java.util.ArrayList;

public class Listener extends AbstractNodeMain {

    Orc orc;
    ServoListener sl;
    private Publisher<ArmMsg> armPub;
    private Subscriber<MotionMsg> motorSub;
    private Subscriber<PositionTargetMsg> motorPosSub;
    private Subscriber<OdometryMsg> motorOdoSub;
    private Subscriber<ArmMsg> armSub;
    private Subscriber<ReverseMsg> motorRevSub;
    private Subscriber<PositionMsg> posSub;
    private Subscriber<WaypointMsg> waypointSub;
    private Subscriber<VelocityMsg> velSub;

    @Override
    public void onStart(final ConnectedNode node) {
        final Log log = node.getLog();

        // Motor Control
        try {

            System.out.println("in main");
            orc = Orc.makeOrc(); // this orc is used only by the servo
                                 // controller, the robotbase makes its own
            if (orc.isSim()) {
                orc.setNode(node);
            }
            RobotBase robot = new RobotBase(node);

            System.out.println("robot base made");

            RobotPositionController robotPositionController = null;
            robotPositionController = new RobotPositionController(
                    new WheelVelocityControllerFF(),
                    new WheelVelocityControllerFF());
            System.out.println("robot position controller created");

            robot.enableMotors(true);
	    robot.setRobotVelocityController(robotPositionController);

	        posSub = node.newSubscriber("/loc/Position", "rss_msgs/PositionMsg");
	        posSub.addMessageListener(new MotorListenerForPosition(robotPositionController));
	        log.info("pos Subscriber created");
	        
	        /* Uncomment to get goal updates from old code.. check topics though..
            motorPosSub = node.newSubscriber("command/Motors", "rss_msgs/PositionTargetMsg");
            motorPosSub.addMessageListener(new MotorListenerForPositionControl(robotPositionController));
            log.info("motor Subscriber created");
            */

            waypointSub = node.newSubscriber("/state/Waypoint", "rss_msgs/WaypointMsg");
            waypointSub.addMessageListener(new MotorListenerForWaypoint(robotPositionController));
            log.info("waypoint Subscriber created");

	    velSub = node.newSubscriber("/state/Velocity", "rss_msgs/VelocityMsg");
	    velSub.addMessageListener(new MotorListenerForVelocity(robotPositionController));
	    log.info("velocity subscriber created");
            
            /* Uncomment to get updates from odometry 
             * 
            motorOdoSub = node.newSubscriber("/rss/odometry", "rss_msgs/OdometryMsg");
            motorOdoSub.addMessageListener(new MotorListenerForOdometry(robotPositionController));
            log.info("motor odometry subscriber created");
             */
            
            motorRevSub = node.newSubscriber("commands/Motors/Reverse", "rss_msgs/ReverseMsg");
            motorRevSub.addMessageListener(new MotorListenerForReverse(robotPositionController));
            log.info("motor reverse subscriber created");

            armPub = node.newPublisher("rss/ArmStatus", "rss_msgs/ArmMsg");

            sl = new ServoListener(orc, armPub,
                    false); // to use safe servos set to true
            // this requires modification of the ServoListener class to have the
            // correct upper and lower bounds
            armSub = node.newSubscriber("command/Arm", "rss_msgs/ArmMsg");
            armSub.addMessageListener(sl);
            log.info("arm subscriber created");

            /*
             * subMonitorThreadBase = new SubMonitorThread(); subMonitorThread =
             * new Thread(this.subMonitorThreadBase); subMonitorThread.start();
             */

        }
        catch (Exception e) {
            e.printStackTrace();
        }

    }

    /*
     * private SubMonitorThread subMonitorThreadBase; private Thread
     * subMonitorThread;
     * 
     * private class SubMonitorThread implements Runnable{
     * 
     * @Override public void run() { while (true){ synchronized(armSub) {
     * java.util.Collection<PublisherIdentifier> publishers = new
     * ArrayList<PublisherIdentifier>(); DefaultSubscriber<?> sub =
     * (DefaultSubscriber<?>)armSub; sub.updatePublishers(publishers);
     * System.out.println(publishers); } try { Thread.sleep(50); } catch
     * (InterruptedException e) { e.printStackTrace(); } } } }
     */

    @Override
    public void onShutdown(Node node) {
        node.shutdown();
        System.out.println(sl.received);
    }

    @Override
    public void onShutdownComplete(Node node) {
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("rss/uorc_listener");
    }
}
