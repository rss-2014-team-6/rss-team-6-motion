package motion.Motor;

import org.ros.message.MessageListener;

import MotorControl.RobotVelocityController;
import rss_msgs.VelocityMsg;

public class MotorListenerForVelocity implements MessageListener<VelocityMsg> {

    private RobotPositionController controller;

    public MotorListenerForVelocity(RobotPositionController rvc) {
        controller = rvc;
    }

    @Override
    public void onNewMessage(VelocityMsg msg) {
	controller.setVelocity(msg.getTranslationVelocity(), msg.getRotationVelocity());
    }

}
