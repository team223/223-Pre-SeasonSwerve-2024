package frc.robot;

import java.util.HashMap;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    // changes swerve speed
    public static final double mult = 1;

    public static final double stickDeadband = 0.1;

  
    public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-
    // PIGEON - false
    // NAVX - true

    /* Drivetrain Constants */
    public static final double trackWidth = Units.inchesToMeters(23);
    public static final double wheelBase = Units.inchesToMeters(23);
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;
    public static final double driveBaseRadius = 19.7;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.12 / 1.0);
    
    public static final double angleGearRatio = (
      //(1.0 / 2.0) * (14.0 / 72.0)
      (7.0/150.0)/1.0
      );
    
      
    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
          new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Compensation */
    public static final double voltageComp = 12.0;
    
    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;
    
    /* Angle Motor PID Values */
    public static final double angleKP = //0.0005;
    0.05;
    public static final double angleKI = //0.0;
    0.0001;
    public static final double angleKD = //0.0;
    0.0025;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = //0.0001;
    //0.16579;
    0.3314;
    public static final double driveKI = //0.0;
    0.0003;
    public static final double driveKD = //0.0;
    0.069;
    public static final double driveKFF = 0.0;
    
    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.084204;
    public static final double driveKV = 2.4615;
    public static final double driveKA = 0.13053;

    //public static final double nearestNinetyKP = 12.17;
    
    /* Drive Motor Conversion Factors
    ratios used for converting between units, between rotations of motor and rotations of output*/
    public static final double drivePositionConversionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor =
        drivePositionConversionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 * angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.8; // Meters per second
    public static final double maxAngularVelocity = 11.5;
    public static final double fineDriveSpeed = 4.6 / 5;
    public static final double maxModuleSpeed = /* temp */ 4.8;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = true;
    public static final boolean angleInvert = true;

    /* Angle Encoder Invert */

    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 10;
      public static final int canCoderID = 23;
      public static final double angleOffset =  
       141.0;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 8;
      public static final int angleMotorID = 9;
      public static final int canCoderID = 22;
      public static final double angleOffset = 
        86.0;
      // public static final double angleOffset = 277.65;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 13;
      public static final int angleMotorID = 16;
      public static final int canCoderID = 24;
      public static final double angleOffset =  
        62.0;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 1;
      public static final int angleMotorID = 5;
      public static final int canCoderID = 21;
      public static final double angleOffset =  
        305.0;
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 7;

    /* Angle Motor PID Values for Auto */
    public static final double AangleKP = //0.05;
    20;
    public static final double AangleKI = //0.0001;
    0;
    public static final double AangleKD = //0.0025;
    0;
    public static final double AangleKFF = 0.0;

    /* Drive Motor PID Values For Auto */
    public static final double AdriveKP = //0.0001;
    //0.16579;
    0.3314;
    public static final double AdriveKI = //0.0;
    0.0003;
    public static final double AdriveKD = //0.0;
    0.069;
    public static final double AdriveKFF = 0.0;

     //Auto balance PID Values 
    

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  
    public static final HashMap<String, Command> eventMap = new HashMap<>();
  }

  

 
}
