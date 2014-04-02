package motion.Motor;

import org.ros.message.MessageListener;

import MotorControl.RobotVelocityController;
import rss_msgs.MotionMsg;
import rss_msgs.PositionTargetMsg;

public class MotorListenerForPositionControl implements MessageListener<PositionTargetMsg> {

    private RobotPositionController controller;

    public MotorListenerForPositionControl(RobotPositionController rvc) {
        controller = rvc;
    }

    @Override
    public void onNewMessage(PositionTargetMsg msg) {

        System.out.println("got pos command: " + msg.getX() + ", " + msg.getY());
        System.out.println("listner received message");
        controller.setGoal(msg.getX(), msg.getY(), msg.getTheta());
    }

}
