package motion.Motor;

import org.ros.message.MessageListener;

import MotorControl.RobotVelocityController;
import rss_msgs.MotionMsg;
import rss_msgs.PositionMsg;

public class MotorListenerForPosition implements MessageListener<PositionMsg> {

    private RobotPositionController controller;

    public MotorListenerForPosition(RobotPositionController rvc) {
        controller = rvc;
    }

    @Override
    public void onNewMessage(PositionMsg msg) {

        // System.out.println("got velocity command: " + msg.translationalVelocity + ", " + msg.rotationalVelocity);
        //System.out.println("odometry listner received message");
        controller.setPose(msg.getX(), msg.getY(), msg.getTheta());
    }

}
