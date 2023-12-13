// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

public class RobotContainer {
  private final SendableChooser<Command> chooser;
  

  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);

  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;


  /* Driver Buttons */  
  private final JoystickButton zeroGyro =
      new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton resetPose =
      new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton xPattern = 
      new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton robotCentric =
      new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton nearest90 = 
      new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  

  /* Subsystems */
  public final Swerve swerve = new Swerve();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve.setDefaultCommand(
        new TeleopSwerve(
            swerve,
            () -> -driver.getRawAxis(translationAxis) * Constants.Swerve.mult,
            () -> -driver.getRawAxis(strafeAxis) * Constants.Swerve.mult,
            () -> -driver.getRawAxis(rotationAxis) * Constants.Swerve.mult,
            () -> robotCentric.getAsBoolean(),
            () -> xPattern.getAsBoolean(),
            () -> driver.getPOV()));
    chooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto:", chooser);
    
    // Configure the button bindings
    configureButtonBindings();

    swerve.resetModuleZeros();


  }


  /** Actions that we want to do when the robot is disabled. */
  public void disabledActions() {
    // s_Swerve.resetModuleZeros();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.toggleOnTrue(new InstantCommand(() -> swerve.zeroGyro(), swerve));
    resetPose.toggleOnTrue(new InstantCommand(() -> swerve.resetPose(new Pose2d()), swerve));
  



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // An ExampleCommand will run in autonomous
   
     
    
    
  public Command getAutonomousCommand() {
    swerve.zeroGyro();
    //swerve.resetOdometry(new Pose2d());
    //PathPlannerPath path = PathPlannerPath.fromPathFile("test");
    Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(chooser.getSelected().getName());

    return new SequentialCommandGroup(

    new InstantCommand(() -> { swerve.zeroGyro(); swerve.resetOdometry(startingPose); }),

    chooser.getSelected()

  );
    

    //return chooser.getSelected();
    //return new TestGyroAuto(swerve);
  }
}
