package frc.robot.util.controllers;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


public class ButtonBox extends Joystick {
	
    private static int GRAB = 9;
    private static int DROP = 10;
    private static int SET_LOW = 1;
    private static int SET_MIDDLE = 2;
    private static int SET_HIGH = 4;
    private static int STOW = 3;
    private static int RIGHTBUMPER = 6;
    private static int LEFTBUMPER = 5;
    private static int START = 8;
    private static int BACK = 7;

	public ButtonBox(int port) {
		super(port);
	}
    public boolean isRightBumper() {
		return this.getRawButton(RIGHTBUMPER);
	}
    public boolean isLeftBumper() {
		return this.getRawButton(LEFTBUMPER);
	}
    public boolean SetLowButton() {
		return this.getRawButton(SET_LOW);
	}
    public boolean SetMidButton() {
		return this.getRawButton(SET_MIDDLE);
	}
    public boolean SetHighButton() {
		return this.getRawButton(SET_HIGH);
	}
    public boolean StowButton() {
		return this.getRawButton(STOW);
	}
    public boolean StartButton(){
      return this.getRawButton(START);
    }

    public boolean RightDpad(){
      return this.RightDpad();
    }
    public boolean BackButton(){
      return this.getRawButton(BACK);
    }


  //public boolean LeftTrigger() {
	//	return this.LeftTrigger();
	//}
  //public boolean RightTrigger() {
	//	return this.RightTrigger();
	//}
	  public Trigger grabButton = new JoystickButton(this, GRAB);
    public Trigger dropButton = new JoystickButton(this, DROP);
    public Trigger setLow = new JoystickButton(this, SET_LOW);
    public Trigger setMiddle = new JoystickButton(this, SET_MIDDLE);
    public Trigger setHigh = new JoystickButton(this, SET_HIGH);    
    public Trigger setStow = new JoystickButton(this, STOW);
    public Trigger ArmUp = new JoystickButton(this, RIGHTBUMPER);
    public Trigger ArmDown = new JoystickButton(this, LEFTBUMPER);
    public Trigger RollerStart = new JoystickButton(this, START);
    public Trigger ReverseRollerStart = new JoystickButton(this, BACK);
    public Trigger UpDpadTrigger = new POVButton(this, 0);
    public Trigger RightDpadTrigger = new POVButton(this, 90);
    public Trigger DownDpadTrigger = new POVButton(this, 180);
    public Trigger LeftDpadTrigger = new POVButton(this, 270);
    //public Trigger leftTrigger = new Trigger(() -> this.LeftTrigger());
    //public Trigger RightTrigger = new Trigger(() -> this.RightTrigger());

}
