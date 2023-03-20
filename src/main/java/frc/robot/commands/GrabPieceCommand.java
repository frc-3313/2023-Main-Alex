package frc.robot.commands;

//import edu.wpi.first.hal.ThreadsJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants;
import frc.robot.subsystems.Grabber;

public class GrabPieceCommand extends CommandBase {
    //private double a_angle;
    public Grabber a_grab;

    public GrabPieceCommand(Grabber grab) {
        // Use requires() here to declare subsystem dependencies
        a_grab = grab;
        // eg. requires(chassis);
        addRequirements(grab);

    }
    
    public void initialize(){
      a_grab.dropPiece();
      a_grab.startRollers();
      
    }
    // Called repeatedly when this Command is scheduled to run
    public void execute() {

    }

    // Make this return true when this Command no longer needs to run execute()
    public boolean isFinished() {

      return true;  
        
    }

    // Called once after isFinished returns true
    public void end() {
      a_grab.grabPiece();
      a_grab.stopRollers();
        //a_arm.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    public void interrupted() {
      // We will be paranoid and do the same clean up if we are interrupted
      a_grab.stopRollers();

      end();
    }
}