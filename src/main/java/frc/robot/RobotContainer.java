package frc.robot;

/* IMPORTS */

// Controllers
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// Subsystems
import frc.robot.subsystems.DriveTrain;
// Constants
import frc.robot.Constants.OIConstants;
// Other
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;

/*
 * RobotContainer.java contains the robot, as well as the bindings that
 * commands are set too on controllers, etc
 * 
 */

public class RobotContainer {
  // SUBSYSTEMS 
  // (add more as they come)
  private final DriveTrain m_robotDrive = new DriveTrain();

  // CONTROLLERS 
  // (please stick to just two)
  XboxController XBOXop = new XboxController(OIConstants.kDriverControllerPortXbox);
  Joystick JOYSTICKop = new Joystick(OIConstants.kDRiverControllerPortStick);

  // CONTROL VALUES 
  // (input channels from the controller imports)
  private final int translationAxis = Joystick.AxisType.kY.value;
  private final int strafeAxis = Joystick.AxisType.kX.value;
  private final int rotationAxis = Joystick.AxisType.kTwist.value;
  
  /* Robot container, with constants, commands, and controllers */

  public RobotContainer() {
    configureButtonBindings();

    // !!SPEED SHOULD BE HANDLED IN CONSTANTS!!
    m_robotDrive.setDefaultCommand(
        new RunCommand(
          () ->
            m_robotDrive.drive(
            -JOYSTICKop.getRawAxis(translationAxis),
            -JOYSTICKop.getRawAxis(strafeAxis),
            -JOYSTICKop.getRawAxis(rotationAxis),
            true, true
            ), m_robotDrive
        )
    );
  }

  /**
   * Method defining button command mappings
   * Use imports from the controllers
   * 
   */
  private void configureButtonBindings() {

    // Stop method that stops all movement while held down
    new JoystickButton(JOYSTICKop, 1)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.test(),
            m_robotDrive));

    new JoystickButton(JOYSTICKop, 3)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.faster(),
            m_robotDrive));

    new JoystickButton(JOYSTICKop, 4)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.slower(),
            m_robotDrive));
  }

  /**
   * Basic command passing an autonomous function to the main Robot class
   * 
   */
  public Command getAutonomousCommand() {
    // basically a placeholder
    return null;
  }
}