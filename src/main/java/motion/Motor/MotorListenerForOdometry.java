package motion.Motor;

import org.ros.message.MessageListener;

import MotorControl.RobotVelocityController;
import rss_msgs.MotionMsg;
import rss_msgs.OdometryMsg;

public class MotorListenerForOdometry implements MessageListener<OdometryMsg> {

    private RobotPositionController controller;

    public MotorListenerForOdometry(RobotPositionController rvc) {
        controller = rvc;
    }

    @Override
    public void onNewMessage(OdometryMsg msg) {

        // System.out.println("got velocity command: " + msg.translationalVelocity + ", " + msg.rotationalVelocity);
        //System.out.println("odometry listner received message");
        //for now, this conflicts with pose -- don't want!  We'll update this once we get to local traversal
	controller.setOdometry(msg.getX(), msg.getY(), msg.getTheta());
    }

}
