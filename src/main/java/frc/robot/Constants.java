package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double stickDeadband2 = 1;

    public static final class Swerve {
        public static final double stickDeadband = 0.1;
    
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
    
        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(20.25);
        public static final double wheelBase = Units.inchesToMeters(26.0);
        public static final double wheelDiameter = Units.inchesToMeters(4.0);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0;

        public static final double driveGearRatio = (5.9/ 1.0);//(6.12 / 1.0); 
        public static final double angleGearRatio = (12.8 / 1.0); // 12.8:1
//12.8 90=270, 
        public static final SwerveDriveKinematics swerveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Voltage Compensation */
        public static final double voltageComp = 12.0;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 20;
        public static final int driveContinuousCurrentLimit = 40;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.01;
        public static final double angleKI = 0.0;
        public static final double angleKD = 0.0;
        public static final double angleKFF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.1;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = 0.667;
        public static final double driveKV = 2.44;
        public static final double driveKA = 0.27;

        /* Drive Motor Conversion Factors */
        public static final double driveConversionPositionFactor =
            (wheelDiameter * Math.PI) / driveGearRatio;
        public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
        public static final double angleConversionFactor = 360.0 / angleGearRatio;

        /* Swerve Profiling Values */
        public static final double maxSpeed = .5; // meters per second
        public static final double maxAngularVelocity = .5;

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final IdleMode driveNeutralMode = IdleMode.kBrake;

        /* Motor Inverts */
        public static final boolean driveInvert = true;
        public static final boolean angleInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;
    
        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
          public static final int driveMotorID = 30;
          public static final int angleMotorID = 31;
          public static final int canCoderID = 20;
          public static final double angleOffset = 18.896;
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    
        /* Front Right Module - Module 1 */
        public static final class Mod1 {
          public static final int driveMotorID = 32;
          public static final int angleMotorID = 33;
          public static final int canCoderID = 23;
          public static final double angleOffset = 316.494;
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
          public static final int driveMotorID = 36;
          public static final int angleMotorID = 37;
          public static final int canCoderID = 21;
          public static final double angleOffset = 116.367;
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    
        /* Back Right Module - Module 3 */
        public static final class Mod3 {
          public static final int driveMotorID = 34;
          public static final int angleMotorID = 35;
          public static final int canCoderID = 22;
          public static final double angleOffset = 8.613;
          public static final SwerveModuleConstants constants =
              new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
      }

    // Drive Train
    public static final int CHARGE_STATION_LEVEL = 2;

//speeds
    public static final double ARM_SPEED = .075;
    public static final double MAX_ARM_SPEED = .3;
    public static final double MAX_WRIST_SPEED = .6;
    public static final double AUTO_ARM_SPEED = .5;
    public static final double AUTO_WRIST_SPEED = 1;

//Arm angles
    public static final double MAX_ARM_ANGLE = 255;
    public static final double STOW_ARM_ANGLE = 250;
    public static final double LOW_ARM_ANGLE = 233;
    public static final double MID_ARM_ANGLE = 145;
    public static final double SHELF_ARM_ANGLE = 133;
    public static final double HIGH_ARM_ANGLE = 127;
    public static final double MIN_ARM_ANGLE = 122;


    //wrist angles through bore encoder
    public static final double MIN_WRIST_ANGLE = 75;
    public static final double STOW_WRIST_ANGLE = 81;
    public static final double LOW_WRIST_ANGLE = 131;
    public static final double MID_WRIST_ANGLE = 185;
    public static final double SHELF_WRIST_ANGLE = 190;
    public static final double HIGH_WRIST_ANGLE = 193;
    public static final double MAX_WRIST_ANGLE = 200;
    // Trajectory
    public static final double MAX_TRAJ_VELOCITY = 4;
    public static final double MAX_TRAJ_CENTRIP_ACC = 3;
    public static final double MAX_TRAJ_ACCELERATION = 3;
    public static final double MAX_VELOCITY = 4;
    
    public static final int SHOULDER_ID = 40;
    public static final int SHOULDER_ID_2 = 41;
    public static final int WRIST_ID = 42;
    public static final int WRIST_ENCODER_ID = 1;

    public static final int GRABBER_ROLLER_ID = 0;
    public static final int GRABBER_DROP = 1;
    public static final int GRABBER_GRAB = 0;
    public static final int GRABBER_CAN = 0;
    public static final int SHOULDER_ENCODER_ID = 0;

    //Controller button id's

    public static int A = 1;
    public static int B = 2;
    public static int X = 3;
    public static int Y = 4;
    public static int LEFTBUMPER = 5;
    public static int RIGHTBUMPER = 6;
    public static int BACK = 7;
    public static int START = 8;
    public static int CLICKRIGHTJOYSTICK = 9;
    public static int CLICKLEFTJOYSTICK = 10;
    
    //Controller Axis id's
    public static int LEFTX = 0;
    public static int LEFTY = 1;
    public static int LEFTTRIGGER = 2;
    public static int RIGHTTRIGGER = 3;
    public static int RIGHTX = 4;
    public static int RIGHTY = 5;
}
