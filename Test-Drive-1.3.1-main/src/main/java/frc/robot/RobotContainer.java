// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
// Controller imports
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// All Drive Train commands and subsystems
import frc.robot.subsystems.DriveTrain;
import frc.robot.commands.DriveTrain.WestCoastDrive;
import frc.robot.commands.DriveTrain.ZOOM;
// All Elevator commands and subsystems
import frc.robot.subsystems.Elevator;
import frc.robot.commands.Elevator.UpAndDown;
// All Claw commands and subsystems
import frc.robot.subsystems.PistonClaw;
import frc.robot.commands.Claw.CloseClaw;
import frc.robot.commands.Claw.OpenClaw;
// All Winch commands and subsystems
import frc.robot.subsystems.Winch;
import frc.robot.commands.Winch.InAndOut;
// Autonomus
import frc.robot.commands.Auto.Autonomous;


public class RobotContainer {
  // Create subsystems
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Elevator m_elevator = new Elevator();
  private final PistonClaw m_claw = new PistonClaw();
  private final Winch m_winch = new Winch();
  private final Autonomous autoCommand;

  // Instantiate driver controller
  public static XboxController driver = new XboxController(Constants.DRIVER_XBOX_PORT);

  // The container for the robot. Contains subsystems, OI devices, and commands.
  public RobotContainer() {
    m_driveTrain.setDefaultCommand(new WestCoastDrive(
      m_driveTrain,
      driver::getLeftY,
      driver::getRightX,
      driver::getXButtonPressed));
    m_winch.setDefaultCommand(new InAndOut(
      m_winch, 
      driver::getRightTriggerAxis,
      driver::getLeftTriggerAxis));
    autoCommand = new Autonomous(m_driveTrain);
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    // Create buttons to link to commands
    JoystickButton xboxControllerLeftStickButton = new JoystickButton(driver, Constants.XBOX_LEFT_STICK_BUTTON);
    JoystickButton xboxControllerLeftBumper = new JoystickButton(driver, Constants.XBOX_LEFT_BUMPER);
    JoystickButton xboxControllerRightBumper = new JoystickButton(driver, Constants.XBOX_RIGHT_BUMPER);
    JoystickButton xboxControllerAButton = new JoystickButton(driver, Constants.XBOX_A_BUTTON);
    JoystickButton xboxControllerBButton = new JoystickButton(driver, Constants.XBOX_B_BUTTON);
    // Link triggers to commands
    xboxControllerLeftBumper.whileHeld(new UpAndDown(m_elevator, -Constants.ELEVATOR_SPEED));
    xboxControllerRightBumper.whileHeld(new UpAndDown(m_elevator, Constants.ELEVATOR_SPEED));
    // Link buttons to Commands
    xboxControllerLeftStickButton.whileHeld(new ZOOM(m_driveTrain));
    xboxControllerAButton.whenPressed(new CloseClaw(m_claw));
    xboxControllerBButton.whenPressed(new OpenClaw(m_claw));
  }

  public Command getAutonomousCommand() {
    /*
    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(
          Constants.kVoltsPos,
          Constants.kVoltsVel,
          Constants.kVoltsAcc),
          DriveTrain.kDriveKinematics,
          10);

    TrajectoryConfig config =
    new TrajectoryConfig(
            Constants.MAX_SPEED_IN_METERS_PER_SECOND,
            Constants.MAX_ACCEL_IN_METERS_PER_SECOND)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveTrain.kDriveKinematics)
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);
          
    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        // Pass config
        config);
    
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        exampleTrajectory,
        m_driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
          Constants.kVoltsPos,
          Constants.kVoltsVel,
          Constants.kVoltsAcc),
          DriveTrain.kDriveKinematics,
          m_driveTrain::getWheelSpeeds,
          new PIDController(Constants.kPDriveVel, 0, 0),
          new PIDController(Constants.kPDriveVel, 0, 0),
        // RamseteCommand passes volts to the callback
        m_driveTrain::tankDriveVolts,
        m_driveTrain);*/

    // A RamseteCommand will run in autonomous
    //return ramseteCommand.andThen(() -> m_driveTrain.tankDriveVolts(0, 0))
    
    return autoCommand;
  }
}
