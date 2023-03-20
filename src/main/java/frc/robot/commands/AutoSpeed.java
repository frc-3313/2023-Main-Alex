package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoSpeed extends CommandBase {    
    private Swerve s_Swerve;
    private double translation; 
    private double strafe; 
    private double rotation;
    private double time;
    private double timeS;
    private Timer m_timer;
    private boolean isRobotOriented;

    public AutoSpeed(Swerve s_Swerve, double translation, double strafe, double rotation, double time, Timer m_timer, boolean isRobotOriented) {
        this.s_Swerve = s_Swerve;
        this.translation = translation; 
        this.strafe = strafe;
        this.rotation = rotation;
        this.time = time;
        this.timeS = 0.0;
        this.m_timer = m_timer;
        this.isRobotOriented = isRobotOriented;
        addRequirements(s_Swerve);
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/

    }

    @Override
    public void initialize() {
        timeS = m_timer.get();
        double translationVal = translation;
        double strafeVal = strafe;
        double rotationVal = rotation;

        s_Swerve.drive(
            new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
            rotationVal * Constants.Swerve.maxAngularVelocity, 
            !isRobotOriented, 
            true
        );
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
 