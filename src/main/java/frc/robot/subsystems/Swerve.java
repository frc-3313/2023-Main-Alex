package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.Constants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.util.drive.DriveUtils;
public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    private final AHRS gyro;
    private double gyroOffset;
    private Pose2d m_pose;
    private Pose2d m_traj_pose;
    private Pose2d m_traj_reset_pose;
    private Pose2d m_traj_offset;


    public Swerve() {
        gyro = new AHRS(SPI.Port.kMXP);
        gyroOffset = 0.0;
        zeroGyro();

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants),
            new SwerveModule(1, Constants.Swerve.Mod1.constants),
            new SwerveModule(2, Constants.Swerve.Mod2.constants),
            new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

        /* By pausing init for a second before setting module offsets, we avoid a bug with inverting motors.
         * See https://github.com/Team364/BaseFalconSwerve/issues/8 for more info.
         */
        Timer.delay(1.0);
        resetModulesToAbsolute();
        m_traj_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
        m_pose = new Pose2d(Units.feetToMeters(0.0), Units.feetToMeters(0.0), new Rotation2d());
        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), m_pose);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
               //  fieldRelative ? 
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getYaw()
                                ))
                                /*:
                                 new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                )*/;
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    
    public void drive(double vx, double vy, double angularVelocity) {
        drive(new ChassisSpeeds(vx, vy, angularVelocity));
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] states = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);
    }
    public ChassisSpeeds getChassisSpeeds() {


        return Constants.Swerve.swerveKinematics.toChassisSpeeds(mSwerveMods[0].getState(), mSwerveMods[1].getState(), mSwerveMods[2].getState(), mSwerveMods[3].getState());
}
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }    

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
        
    }
    public void resetOdometry(Pose2d startPose, Rotation2d startGyro) {
        swerveOdometry.resetPosition(startGyro,
                        getModulePositions(),
                        startPose);
        gyroOffset = startPose.getRotation().getDegrees() - startGyro.getDegrees();
    }
    public void resetTrajectoryPose(Pose2d startPose) {
        m_traj_reset_pose = swerveOdometry.getPoseMeters();
        m_traj_offset = startPose;
        gyroOffset = startPose.getRotation().getDegrees();
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro(){
        gyro.zeroYaw();
    }
    
    public double getFieldOffset() {
        return gyroOffset;
    }

    public Rotation2d getYaw() {
        return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw() + gyroOffset) : Rotation2d.fromDegrees(gyro.getYaw() + gyroOffset);
    }

    public double getRoll() {
        return gyro.getRoll();
    }
    public double getYaw2() {
        return gyro.getYaw();
    }
    public double getPitch() {
        return gyro.getPitch();
    }
    public void setGyroOffset(double gyroOffset) {
        this.gyroOffset = gyroOffset;
    }
    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }
    public Pose2d getTrajectoryOdometryPose() {
        return m_traj_pose;
    }
    public void updateOdometry(){
        m_pose = swerveOdometry.update(getYaw(), getModulePositions());  
        if (m_traj_reset_pose == null || m_traj_offset == null) {
            m_traj_pose = m_pose;
        } else {
            Pose2d reset_relative_pose = m_pose.relativeTo(m_traj_reset_pose);
            m_traj_pose = DriveUtils.relativeToReverse(reset_relative_pose, m_traj_offset);
        }
    }
    @Override
    public void periodic(){
        updateOdometry();
        SmartDashboard.putNumber("EncoderReadingFL", mSwerveMods[0].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("EncoderReadingFR", mSwerveMods[1].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("EncoderReadingBL", mSwerveMods[2].getAbsoluteEncoderRad());
        SmartDashboard.putNumber("EncoderReadingBR", mSwerveMods[3].getAbsoluteEncoderRad());
        SmartDashboard.putString("RobotHeading", this.getYaw().toString());
        SmartDashboard.putNumber("Robot Roll", this.getPitch());
        SmartDashboard.putNumber("Robot Yaw", this.getYaw2());
        
        for(SwerveModule mod : mSwerveMods){
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);    
        }
    }

    public void stop() {
        drive(
            new Translation2d(0, 0), 
            0, 
            !false, 
            true);
       }
}