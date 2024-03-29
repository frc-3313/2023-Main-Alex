package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class ArmWristCommand extends CommandBase {
    private double a_angle;
    private double w_angle;
    public Wrist a_Wrist;
    public Arm a_arm;
    public double startAngle;
    public double a_armSpeed;
    public double w_wristSpeed;
    public boolean Arm_first;
    public boolean Arm_Moved = false;
    public boolean Wrist_Moved = false;

    public ArmWristCommand(Arm arm, double sAngle, double armSpeed, Wrist wrist, double wAngle, double wristSpeed) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
        addRequirements(arm, wrist);
        a_angle = sAngle;
        a_arm = arm;
        a_Wrist = wrist;
        w_angle = wAngle;
        a_armSpeed = armSpeed;
        w_wristSpeed = wristSpeed;

    }

    public void initialize(){
    //  arm is going up
      a_arm.setArmSpeed(a_armSpeed);
      a_Wrist.setWristSpeed(w_wristSpeed);
      if(a_arm.getDegrees() > a_angle){
        a_arm.setSetpoint(a_angle);
        Arm_first = true;
        Arm_Moved = true;
      }
      else{
        a_Wrist.setSetpoint(w_angle);
        Arm_first = false;
        Wrist_Moved = true;
      }
    }
    // Called repeatedly when this Command is scheduled to run
    public void execute() {
      //if(Arm_first && a_arm.atSetpoint()){
      if(Arm_first && ((a_arm.getDegrees() < 160) || a_arm.atSetpoint())){
        a_Wrist.setSetpoint(w_angle);
        Wrist_Moved = true;
      }
      //else if(!Arm_first && a_Wrist.atSetpoint()){
      else if(!Arm_first && ((a_Wrist.getDegrees() > 100) || a_Wrist.atSetpoint())){
        a_arm.setSetpoint(a_angle);
        Arm_Moved = true;
      }

    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {
      // Stop once we've moved to or past the end distance
      return a_arm.atSetpoint() && a_Wrist.atSetpoint() && Arm_Moved && Wrist_Moved;
        
    }

    // Called once after isFinished returns true
    public void end() {
        a_arm.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
      // We will be paranoid and do the same clean up if we are interrupted
      end();
    }
}