package Common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** 
 * <p>Creates a ThrustMaster specific {@link Joystick} including methods for every button and axis</p>
 */
public class ThrustMaster  {

    Joystick joy;

//==============================================================================
//=============================== Joystick IDs ===================================
    private final int TRIGGER = 1;
    private final int MIDDLE = 2;
    private final int LEFT = 3;
    private final int RIGHT = 4;

    private final int LEFT_FRONT_LEFT = 5;
    private final int LEFT_FRONT_MIDDLE = 6;
    private final int LEFT_FRONT_RIGHT = 7;
    private final int LEFT_BACK_RIGHT = 8;
    private final int LEFT_BACK_MIDDLE = 9;
    private final int LEFT_BACK_LEFT = 10;

    private final int RIGHT_FRONT_RIGHT = 11;
    private final int RIGHT_FRONT_MIDDLE = 12;
    private final int RIGHT_FRONT_LEFT = 13;
    private final int RIGHT_BACK_LEFT = 14;
    private final int RIGHT_BACK_MIDDLE = 15;
    private final int RIGHT_BACK_RIGHT = 16;

    private final int xAxis = 1;
    private final int yAxis = 2;
    private final int zAxis = 3;
    private final int throttleAxis = 4;
    private final int hatXAxis = 5;
    private final int hatYAxis = 6;


//==============================================================================
//============================== Constructor ===================================

    public ThrustMaster(int port) {
        this.joy = new Joystick(port);
    }

//==============================================================================
//============================ Create Buttons ==================================

    JoystickButton trigger = new JoystickButton(joy, TRIGGER);
    JoystickButton middle = new JoystickButton(joy, MIDDLE);
    JoystickButton left = new JoystickButton(joy, LEFT);
    JoystickButton right = new JoystickButton(joy, RIGHT);
    JoystickButton leftFrontLeft = new JoystickButton(joy, LEFT_FRONT_LEFT);
    JoystickButton leftFrontMiddle = new JoystickButton(joy, LEFT_FRONT_MIDDLE);
    JoystickButton leftFrontRight = new JoystickButton(joy, LEFT_FRONT_RIGHT);
    JoystickButton leftBackRight = new JoystickButton(joy, LEFT_BACK_RIGHT);
    JoystickButton leftBackMiddle = new JoystickButton(joy, LEFT_BACK_MIDDLE);
    JoystickButton leftBackLeft = new JoystickButton(joy, LEFT_BACK_LEFT);
    JoystickButton rightFrontRight = new JoystickButton(joy, RIGHT_FRONT_RIGHT);
    JoystickButton rightFrontMiddle = new JoystickButton(joy, RIGHT_FRONT_MIDDLE);
    JoystickButton rightFrontLeft = new JoystickButton(joy, RIGHT_FRONT_LEFT);
    JoystickButton rightBackLeft = new JoystickButton(joy, RIGHT_BACK_LEFT);
    JoystickButton rightBackMiddle = new JoystickButton(joy, RIGHT_BACK_MIDDLE);
    JoystickButton rightBackRight = new JoystickButton(joy, RIGHT_BACK_RIGHT);



//==============================================================================
//================================ Getters =====================================

    public Joystick getJoy() {
        return joy;
    }
    public JoystickButton getTrigger() {
        return trigger;
    }
    public JoystickButton getMiddle() {
        return middle;
    }
    public JoystickButton getLeft() {
        return left;
    }
    public JoystickButton getRight() {
        return right;
    }
    public JoystickButton getLeftFrontLeft() {
        return leftFrontLeft;
    }
    public JoystickButton getLeftFrontMiddle() {
        return leftFrontMiddle;
    }
    public JoystickButton getLeftFrontRight() {
        return leftFrontRight;
    }
    public JoystickButton getLeftBackRight() {
        return leftBackRight;
    }
    public JoystickButton getLeftBackMiddle() {
        return leftBackMiddle;
    }
    public JoystickButton getLeftBackLeft() {
        return leftBackLeft;
    }
    public JoystickButton getRightFrontRight() {
        return rightFrontRight;
    }
    public JoystickButton getRightFrontMiddle() {
        return rightFrontMiddle;
    }
    public JoystickButton getRightFrontLeft() {
        return rightFrontLeft;
    }
    public JoystickButton getRightBackLeft() {
        return rightBackLeft;
    }
    public JoystickButton getRightBackMiddle() {
        return rightBackMiddle;
    }
    public JoystickButton getRightBackRight() {
        return rightBackRight;
    }    
    public double getX() {
        return joy.getRawAxis(xAxis);
    }
    public double getY() {
        return joy.getRawAxis(yAxis);
    }
    public double getZ() {
        return joy.getRawAxis(zAxis);
    }
    public double getThrottle() {
        return joy.getRawAxis(throttleAxis);
    }
    public double getHatX() {
        return joy.getRawAxis(hatXAxis);
    }
    public double getHatY() {
        return joy.getRawAxis(hatYAxis);
    }

}
