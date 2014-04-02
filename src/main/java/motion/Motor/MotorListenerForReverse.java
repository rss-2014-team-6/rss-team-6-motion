package motion.Motor;

import org.ros.message.MessageListener;

import MotorControl.RobotVelocityController;
import rss_msgs.MotionMsg;
import rss_msgs.ReverseMsg;

public class MotorListenerForReverse implements MessageListener<ReverseMsg> {

    private RobotPositionController controller;

    public MotorListenerForReverse(RobotPositionController rvc) {
        controller = rvc;
    }

    @Override
    public void onNewMessage(ReverseMsg msg) {
        controller.setReverse(msg.getReverse());
    }

}
