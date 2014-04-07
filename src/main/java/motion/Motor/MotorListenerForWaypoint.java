package motion.Motor;

import org.ros.message.MessageListener;

import MotorControl.RobotVelocityController;
import rss_msgs.MotionMsg;
import rss_msgs.WaypointMsg;

public class MotorListenerForWaypoint implements MessageListener<WaypointMsg> {

    private RobotPositionController controller;

    public MotorListenerForWaypoint(RobotPositionController rvc) {
        controller = rvc;
    }

    @Override
    public void onNewMessage(WaypointMsg msg) {

        // System.out.println("got velocity command: " + msg.translationalVelocity + ", " + msg.rotationalVelocity);
        //System.out.println("odometry listner received message");
        controller.setGoal(msg.getX(), msg.getY(), msg.getTheta());
    }

}

