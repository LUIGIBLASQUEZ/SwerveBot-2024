// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//CONTROLLER IMPORTS
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveTrain;
import frc.robot.Constants.OIConstants;
//ETC IMPORTS (STILL IMPORTANT)
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/*
 * RobotContainer manages the controllers, basically
 * (i hope no one asks me questions about it because its very blunt)
 * 
 */
public class RobotContainer {
  // SUBSYSTEMS (add more later for whatever subsystems are added)
  private final DriveTrain m_robotDrive = new DriveTrain();

  // CONTROLLERS (id recommend just stick to these two)
  XboxController XBOXop = new XboxController(OIConstants.kDriverControllerPortXbox);
  Joystick JOYSTICKop = new Joystick(OIConstants.kDRiverControllerPortStick);

  // CONTROL VALUES (ids pertaining to the controller imports)
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kTwist.value;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // !!SPEED SHOULD BE HANDLED IN CONSTANTS!!
    
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
          () ->
            m_robotDrive.drive(
            -JOYSTICKop.getRawAxis(translationAxis),
            -JOYSTICKop.getRawAxis(strafeAxis),
            -JOYSTICKop.getRawAxis(rotationAxis),
            true, true
            ), m_robotDrive
            ));
        /* 
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(JOYSTICKop.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(JOYSTICKop.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(JOYSTICKop.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
        */
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(JOYSTICKop, 1)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.stop(),
            m_robotDrive));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // basically a placeholder
    return null;
  }
}