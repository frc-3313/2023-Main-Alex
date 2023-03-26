package frc.robot.commands;

import frc.robot.Constants.*;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoSpeed extends CommandBase {    
    private Drivetrain s_Swerve;
    private double translation; 
    private double strafe; 
    private double rotation;
    private double time;
    private double timeS;
    private Timer m_timer;

    public AutoSpeed(Drivetrain s_Swerve, double translation, double strafe, double rotation, double time, Timer m_timer) {
        this.s_Swerve = s_Swerve;
        this.translation = translation; 
        this.strafe = strafe;
        this.rotation = rotation;
        this.time = time;
        this.timeS = 0.0;
        this.m_timer = m_timer;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

    }

    @Override
    public void initialize() {
        timeS = m_timer.get();
        double translationVal = translation * AutoConstants.kMaxSpeed;
        double strafeVal = strafe * AutoConstants.kMaxSpeed;
        double rotationVal = rotation * AutoConstants.kMaxAngularSpeed;

        s_Swerve.drive(translationVal, strafeVal, rotationVal, true);
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
 