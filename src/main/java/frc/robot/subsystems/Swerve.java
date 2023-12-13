package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods = new SwerveModule[] {
    new SwerveModule(0, Constants.Swerve.Mod0.constants),
    new SwerveModule(1, Constants.Swerve.Mod1.constants),
    new SwerveModule(2, Constants.Swerve.Mod2.constants),
    new SwerveModule(3, Constants.Swerve.Mod3.constants)
  };
  public AHRS navx;
  public Field2d field;
  

  public Swerve() {

    navx = new AHRS(SPI.Port.kMXP, (byte) 200);

    // zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), getModulePositions(), new Pose2d(0, 0, getYaw()));

    field = new Field2d();

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getRobotRelativeSpeeds, 
      this::setChassisSpeeds, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(Constants.AutoConstants.AdriveKP,Constants.AutoConstants.AdriveKI,Constants.AutoConstants.AdriveKD),
        new PIDConstants(Constants.AutoConstants.AangleKP,Constants.AutoConstants.AangleKI,Constants.AutoConstants.AangleKD),
        Constants.Swerve.maxModuleSpeed,
        Constants.Swerve.driveBaseRadius,
        new ReplanningConfig(true,false)
      ), 
      this
    );
  }    

 


  public void fineDrive(int angle) {
    drive(
      new Translation2d(
        Math.cos(Math.toRadians(angle)) * Constants.Swerve.fineDriveSpeed, 
        Math.sin(Math.toRadians(angle)) * Constants.Swerve.fineDriveSpeed), 
        0, 
        true,  
        false);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    
      SwerveModuleState[] swerveModuleStates =
        Constants.Swerve.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);
      

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

   public void resetPose(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
  }

  public void resetModuleZeros() { 
    for (SwerveModule mod : mSwerveMods) {
      mod.resetToAbsolute();
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    ChassisSpeeds chassisSpeeds = Constants.Swerve.swerveKinematics.toChassisSpeeds(getStates());
    return chassisSpeeds;
  }

  
  
  //Returns the states of all of the swerve modules within a list
  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public void setStates(SwerveModuleState[] desiredStates) {
    setStates(desiredStates, false);
  }

  public void setStates(SwerveModuleState[] desiredStates, boolean overrideSpeedLimit) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false, overrideSpeedLimit);
    }
  }

  public void zeroGyro() {
    navx.zeroYaw();
  }

  public Rotation2d getYaw() {
    if (Constants.Swerve.invertGyro) {
      // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes
      // the angle increase.
      return Rotation2d.fromDegrees((360.0 - navx.getYaw()));
    } else {
      return Rotation2d.fromDegrees(navx.getYaw()); 
    }
  }

  // Get the position and rotation of each swerve module for odometry
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getPosition();
    }
    return states;
  }

  
  // Sets the speeds of each swerve module based off a ChassisSpeeds objects
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    SwerveModuleState[] desiredStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void updateOdometry(){
    swerveOdometry.update(getYaw(), getModulePositions());
  }

  @Override
  public void periodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry tv = table.getEntry("tv");

    double target = tv.getDouble(0.0);
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);
    double skew = ts.getDouble(0.0);

    // if (limelighting) {
    //   LimelightResults llResults = LimelightHelpers.getLatestResults("");
    // if (llResults.targetingResults.targets_Fiducials.length > 1) {
    //   double[] pose; // Translation X,Y,Z Rotation X,Y,Z
    //   if (DriverStation.getAlliance() == Alliance.Blue) {
    //     pose = llResults.targetingResults.botpose_wpiblue;
    //   } else {
    //     pose = llResults.targetingResults.botpose_wpired;
    //   }

    //   Translation2d pos = new Translation2d(pose[0], pose[1]);
    //   swerveOdometry.resetPosition(
    //     getYaw(), 
    //     getModulePositions(), 
    //     new Pose2d(pos, getYaw())
    //   );
    // }
    //}

    //FOr displaying in shuffleboard
    
    updateOdometry();

    field.setRobotPose(swerveOdometry.getPoseMeters());
    
    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getCanCoder().getDegrees());
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Integrated", mod.getState().angle.getDegrees() % 360);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Desired", mod.desiredState.angle.getDegrees() % 360);
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
    }
     
    SmartDashboard.putData("Gyro", navx);
    SmartDashboard.putData("Field", field);
    SmartDashboard.putNumber("Gyro Degrees", getYaw().getDegrees());
    SmartDashboard.putNumber("Pitch", navx.getPitch());
    
    
    SmartDashboard.putNumber("Pose Y", swerveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Pose X", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Pose Rotate", swerveOdometry.getPoseMeters().getRotation().getDegrees());
/* 
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightSkew", skew);
    SmartDashboard.putNumber("LimelightTarget", target);*/
  }
}
