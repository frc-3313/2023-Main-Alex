package frc.robot.commands.autos;

//import frc.robot.Constants;
import frc.robot.subsystems.Grabber;

//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoOpenGrabber extends CommandBase {    
    //private double speedVal;
    //private boolean wantGrip;
    private Timer m_timer;
    private double time;
    private double timeS;
    private Grabber s_Grabber; 

    public AutoOpenGrabber(Grabber s_Grabber, double time, Timer m_timer) {
        this.s_Grabber = s_Grabber;
 
        this.m_timer = m_timer;
        this.time = time;
        this.timeS = 0.0;
        addRequirements(s_Grabber);
    } 

    @Override
    public void initialize() {
        timeS = m_timer.get();
    }

    @Override
    public void execute() {
        s_Grabber.grabPiece();
    }

    @Override
    public boolean isFinished() {
        if(m_timer.get() - timeS >= time) {
            return true;
        }
        else {
            return false;
        }

    }

}
 