package frc.robot.commands.drive;

//import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends CommandBase {
    private final Swerve m_drivetrainSubsystem;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;

    public SwerveDriveCommand(Swerve drivetrainSubsystem,
                               DoubleSupplier translationXSupplier,
                               DoubleSupplier translationYSupplier,
                               DoubleSupplier rotationSupplier) {
        this.m_drivetrainSubsystem = drivetrainSubsystem;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
            m_drivetrainSubsystem.drive(
            new Translation2d(m_translationXSupplier.getAsDouble(), m_translationYSupplier.getAsDouble()), 
            m_rotationSupplier.getAsDouble() * Constants.Swerve.maxAngularVelocity, 
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