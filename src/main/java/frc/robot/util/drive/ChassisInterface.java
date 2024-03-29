package frc.robot.util.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public interface ChassisInterface {
   // public void drive(VelocityCommand command);
    public void drive(double vx, double vy, double angularVelocity);
    public void stop();
    public void resetPosition(Pose2d pose);
    public Pose2d getOdometryPose();
    public ChassisSpeeds getChassisSpeeds();
}
