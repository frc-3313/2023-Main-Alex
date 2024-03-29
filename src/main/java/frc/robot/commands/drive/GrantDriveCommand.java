package frc.robot.commands.drive;

//import edu.wpi.first.hal.ThreadsJNI;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class GrantDriveCommand extends CommandBase {
    private final Swerve m_drivetrainSubsystem;

    private final DoubleSupplier m_throttleSupplier;
    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final BooleanSupplier m_halfSpeed;
    private final BooleanSupplier m_quarterSpeed;

    private double m_lastHeading = 0;

    private boolean m_wasRotating = false;

    public GrantDriveCommand(Swerve drivetrainSubsystem,
                               DoubleSupplier throttleSupplier,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier,
                               BooleanSupplier halfSpeed,
                               BooleanSupplier quarterSpeed) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_throttleSupplier = throttleSupplier;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_halfSpeed = halfSpeed;
        this.m_quarterSpeed = quarterSpeed;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        double heading = Math.atan2(-m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble());
        heading -= Math.toRadians(m_drivetrainSubsystem.getFieldOffset());
        double rotation = m_rotationSupplier.getAsDouble();
        double throttle = m_throttleSupplier.getAsDouble();
        boolean pointing = Math.abs(m_translationXSupplier.getAsDouble()) > 0.25 || Math.abs(m_translationYSupplier.getAsDouble()) > 0.25;
        if(m_quarterSpeed.getAsBoolean()) {
            throttle = throttle/4;
        } 
        else if(m_halfSpeed.getAsBoolean()) {
            throttle = throttle/2;
        }
        if (!pointing) {
            heading = m_lastHeading;
        } else {
            m_lastHeading = heading;
        }

        if (Math.abs(throttle) < 0.001 && Math.abs(rotation) < 0.001) {
            if (pointing) {
                m_wasRotating = false;
                throttle = 0.001;
            } else if (m_wasRotating) {
                rotation = 0.001;
            } else {
                throttle = 0.001;
            }
        } else {
            m_wasRotating = Math.abs(rotation) >= 0.001;
        }

        double vx = throttle * Math.cos(heading);
        double vy = throttle * Math.sin(heading);

        m_drivetrainSubsystem.drive(
            new Translation2d(vx, vy), 
            rotation * Constants.Swerve.maxAngularVelocity, 
            !false, 
            true);

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(
            new Translation2d(0, 0), 
            0, 
            !false, 
            true);
    }
}