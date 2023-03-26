package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public final class Constants {
    public static final class CurrentLimit {
        public static final int kTranslation = 30;
        public static final int kRotation = 25;
    }
    public static final double stickDeadband = 0.1;
    public static final double stickDeadband2 = 1;
          // Trajectory
    public static final double MAX_TRAJ_VELOCITY = 4;
    public static final double MAX_TRAJ_CENTRIP_ACC = 3;
    public static final double MAX_TRAJ_ACCELERATION = 3;
    public static final double MAX_VELOCITY = 4;

    public static final class AutoConstants {
        public static final double kMaxAcceleration = 2.50;
        public static final double kMaxSpeed = 3.25; // Maximum Sustainable Drivetrain Speed under Normal Conditions &
                                                     // Battery, Robot will not exceed this speed in closed loop control
        public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly
        public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                               // but spinning fast is not particularly useful or driver
                                                               // friendly
        public static final double kPXController = 3.0;
        public static final double kPYController = 3.0;
        public static final double kPThetaController = 3.0;
    
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeed, kMaxAngularAccel); // Creates a trapezoidal motion for the auto rotational commands
      }
   
    //speeds
        public static final double ARM_SPEED = .075;
        public static final double MAX_ARM_SPEED = .3;
        public static final double MAX_WRIST_SPEED = .6;

    //wrist angles through bore encoder
    public static final double MIN_WRIST_ANGLE = 65;
    public static final double STOW_WRIST_ANGLE = 75;
    public static final double LOW_WRIST_ANGLE = 118;
    public static final double MID_WRIST_ANGLE = 185;
    public static final double SHELF_WRIST_ANGLE = 183;
    public static final double HIGH_WRIST_ANGLE = 180;
    public static final double MAX_WRIST_ANGLE = 210;

    
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

 /**
   * Static method containing all Drivetrain constants
   */
  public static final class DriveConstants {
    public static final int kFrontLeftDriveMotorPort = 19; // CANID of the Translation SparkMAX
    public static final int kFrontRightDriveMotorPort = 4; // CANID of the Translation SparkMAX
    public static final int kBackLeftDriveMotorPort = 6; // CANID of the Translation SparkMAX
    public static final int kBackRightDriveMotorPort = 2; // CANID of the Translation SparkMAX

    public static final int kFrontLeftTurningMotorPort = 18; // CANID of the Rotation SparkMAX
    public static final int kFrontRightTurningMotorPort = 3; // CANID of the Rotation SparkMAX
    public static final int kBackLeftTurningMotorPort = 7; // CANID of the Rotation SparkMAX
    public static final int kBackRightTurningMotorPort = 1; // CANID of the Rotation SparkMAX

    public static final int kFrontLeftTurningEncoderPort = 0; // Analog Port of the Module Absolute Encoder
    public static final int kFrontRightTurningEncoderPort = 1; // Analog Port of the Module Absolute Encoder
    public static final int kBackLeftTurningEncoderPort = 3; // Analog Port of the Module Absolute Encoder
    public static final int kBackRightTurningEncoderPort = 2; // Analog Port of the Module Absolute Encoder

    public static final double kFrontLeftOffset = 1.3632+0.0122; // Encoder Offset in Radians
    public static final double kFrontRightOffset = -0.3005-0.195-0.0115; // Encoder Offset in Radians
    public static final double kBackLeftOffset = 0.3192+0.0332; // Encoder Offset in Radians
    public static final double kBackRightOffset = -5.9208+0.008; // Encoder Offset in Radians

    // Drive motor PID is best done on the roboRIO currently as the SparkMAX does
    // not allow for static gain values on the PID controller,
    // these are necessary to have high accuracy when moving at extremely low RPMs
    // public static final double[] kFrontLeftTuningVals = {0.0120,0.2892,0.25,0};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kFrontRightTuningVals = {0.0092,0.2835,0.25,1};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackLeftTuningVals = {0.0142,0.2901,0.25,2};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}
    // public static final double[] kBackRightTuningVals = {0.0108,0.2828,0.25,3};
    // //{Static Gain, FeedForward, Proportional Gain, ModuleID for Tuning}

    public static final double[] kFrontLeftTuningVals = { 0.0150, 0.2850, 0.25, 0 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}
    public static final double[] kFrontRightTuningVals = { 0.0150, 0.2850, 0.25, 1 }; // {Static Gain, FeedForward,
                                                                                      // Proportional Gain, ModuleID for
                                                                                      // Tuning}
    public static final double[] kBackLeftTuningVals = { 0.0150, 0.2850, 0.25, 2 }; // {Static Gain, FeedForward,
                                                                                    // Proportional Gain, ModuleID for
                                                                                    // Tuning}
    public static final double[] kBackRightTuningVals = { 0.0150, 0.2850, 0.25, 3 }; // {Static Gain, FeedForward,
                                                                                     // Proportional Gain, ModuleID for
                                                                                     // Tuning}

    // NOTE: 2910 Swerve the wheels are not directly under the center of rotation
    // (Take into consideration when measuring)
    public static final double kWheelBaseWidth = 0.5588; // Center distance in meters between right and left wheels on
                                                         // robot
    public static final double kWheelBaseLength = 0.6446; // Center distance in meters between front and back wheels on
                                                          // robot

    // Because the swerve modules poisition does not change, define a constant
    // SwerveDriveKinematics for use throughout the code
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2),
        new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2));

    public static final double kMaxAcceleration = 3.0;
    public static final double kMaxSpeedMetersPerSecond = 3.25; // Maximum Sustainable Drivetrain Speed under Normal
                                                                // Conditions & Battery, Robot will not exceed this
                                                                // speed in closed loop control
    public static final double kMaxAngularSpeed = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly
    public static final double kMaxAngularAccel = Math.PI; // Maximum Angular Speed desired. NOTE: Robot can exceed this
                                                           // but spinning fast is not particularly useful or driver
                                                           // friendly

    public static final double kInnerDeadband = 0.10; // This value should exceed the maximum value the analog stick may
                                                      // read when not in use (Eliminates "Stick Drift")
    public static final double kOuterDeadband = 0.98; // This value should be lower than the analog stick X or Y reading
                                                      // when aimed at a 45deg angle (Such that X and Y are are
                                                      // maximized simultaneously)
    public static final double kTranslationSlew = 1.55;
    public static final double kRotationSlew = 3.00;

    // Minimum allowable rotation command (in radians/s) assuming user input is
    // squared using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinRotationCommand = DriveConstants.kMaxAngularSpeed
        * Math.pow(DriveConstants.kInnerDeadband, 2);
    // Minimum allowable tranlsation command (in m/s) assuming user input is squared
    // using quadraticTransform, this value is always positive and should be
    // compared agaisnt the absolute value of the drive command
    public static final double kMinTranslationCommand = DriveConstants.kMaxSpeedMetersPerSecond
        * Math.pow(DriveConstants.kInnerDeadband, 2);

    public static final double[] kKeepAnglePID = { 0.500, 0, 0 }; // Defines the PID values for the keep angle PID

  }

  /**
   * Static method containing all Swerve Module constants
   */
  public static final class ModuleConstants {
    public static final double kTranslationRampRate = 3.0; // Units of %power/s, ie 4.0 means it takes 0.25s to reach
                                                           // 100% power from 0%
    private static final double kTranslationGearRatio = 6.12; // Overall gear ratio of the swerve module
    private static final double kWheelDiameter = 0.1016; // Wheel Diameter in meters, may need to be
                                                                         // experimentally determined due to compliance
                                                                         // of floor/tread material

    public static final double kVelocityFactor = (1.0 / kTranslationGearRatio / 60.0) * kWheelDiameter * Math.PI; // Calculates
                                                                                                                  // the
                                                                                                                  // conversion
                                                                                                                  // factor
                                                                                                                  // of
                                                                                                                  // RPM
                                                                                                                  // of
                                                                                                                  // the
                                                                                                                  // translation
                                                                                                                  // motor
                                                                                                                  // to
                                                                                                                  // m/s
                                                                                                                  // at
                                                                                                                  // the
                                                                                                                  // floor

    // NOTE: You shoulds ALWAYS define a reasonable current limit when using
    // brushless motors
    // due to the extremely high stall current avaialble

    public static final double[] kTurnPID = { 0.600, 0, 0 }; // Defines the PID values for rotation of the serve
                                                             // modules, should show some minor oscillation when no
                                                             // weight is loaded on the modules
                                                                     /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor =
        (kWheelDiameter * Math.PI) / kTranslationGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleGearRatio = (12.8 / 1.0);
    public static final double angleConversionFactor = 360.0 / angleGearRatio;
  }

  /**
   * Static method containing all User I/O constants
   */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // When making use of multiple controllers for drivers each
                                                       // controller will be on a different port
    public static final int kOperatorControllerPort = 1; // When making use of multiple controllers for drivers each
                                                         // controller will be on a different port
  }

  /**
   * Static method containing all Global constants
   */
  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
  
}
