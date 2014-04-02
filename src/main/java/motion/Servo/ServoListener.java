package motion.Servo;

import orc.Orc;
import orc.Servo;

import org.ros.message.MessageListener;
import rss_msgs.ArmMsg;
import org.ros.node.topic.Publisher;

/**
 * <p>
 * Implements a message listener to listen for ROS arm commands. Also publishes
 * data about the current arm status at 20Hz. Information about which channel it
 * publishes to is found in the Listener class. This class will need to be
 * modified for a specific robot in order to set the correct bounds on the arm
 * motion.
 * </p>
 * 
 * @author rss-staff
 * 
 */

public class ServoListener implements MessageListener<ArmMsg> {

    private Orc orc;
    private Servo bigServo;
    private Servo gripperServo;
    private Servo wristServo;
    private ArmPubThread armThreadBase;
    private Thread pubThread;

    public int received = 0;

    public ServoListener(Orc orc, Publisher<ArmMsg> armPublisher, boolean safe) {
        this.orc = orc;
        if (safe) {
            bigServo = new SafeServo(orc, 5, 552, 2200, 2200);// start in the upright position
            gripperServo = new SafeServo(orc, 2, 1540, 1750, 1750);// start open
            wristServo = new SafeServo(orc, 4, 1100, 1600, 1600);// start parallel to the ground
        }
        else {
            bigServo = new Servo(orc, 5, 0, 0, 0, 0);
            gripperServo = new Servo(orc, 2, 0, 0, 0, 0);
            wristServo = new Servo(orc, 4, 0, 0, 0, 0);
        }
        this.armThreadBase = new ArmPubThread(armPublisher);
        pubThread = new Thread(this.armThreadBase);
        pubThread.start();
    }

    @Override
    public void onNewMessage(ArmMsg msg) {
        long bigPWM = msg.getPwms()[0];
        long wristPWM = msg.getPwms()[1];
        long gripperPWM = msg.getPwms()[2];

        if (bigPWM == 0) {
            bigServo.idle();
        }
        else {
            bigServo.setPulseWidth((int) bigPWM);
        }

        if (wristPWM == 0) {
            wristServo.idle();
        }
        else {
            wristServo.setPulseWidth((int) wristPWM);
        }

        if (gripperPWM == 0) {
            gripperServo.idle();
        }
        else {
            gripperServo.setPulseWidth((int) gripperPWM);
        }

        this.armThreadBase.update(msg);
        // System.out.println("got servo message: " + bigPWM + ", " + gripperPWM + ", " + wristPWM);
    }

    /**
     * <p>
     * Class that enables this servo listener to publish the current state of
     * the arm. The run loop publishes arm messages at 20 Hz.
     * </p>
     * 
     * @author Dylan Hadfield-Menell
     * 
     */
    private class ArmPubThread implements Runnable {

        private Publisher<ArmMsg> armPub;
        private ArmMsg msg;

        private ArmPubThread(Publisher<ArmMsg> pub) {
            this.armPub = pub;
            msg = pub.newMessage();
            msg.setPwms(new long[8]);
        }

        private void update(ArmMsg msg) {
            synchronized (this.msg) {
                this.msg = msg;
            }
        }

        @Override
        public void run() {
            while (true) {
                synchronized (this.msg) {
                    armPub.publish(this.msg);
                }
                try {
                    Thread.sleep(50);
                }
                catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
        }

    }

}
