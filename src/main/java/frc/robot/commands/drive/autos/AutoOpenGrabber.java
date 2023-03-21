package frc.robot.commands.drive.autos;

//import frc.robot.Constants;
import frc.robot.subsystems.Grabber;

//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoOpenGrabber extends CommandBase {    

    private Grabber s_Grabber; 

    public AutoOpenGrabber(Grabber s_Grabber) {
        this.s_Grabber = s_Grabber;
        addRequirements(s_Grabber);
    } 

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        s_Grabber.dropPiece();
    }

    @Override
    public boolean isFinished() {
        return true;

    }

}
 