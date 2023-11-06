package frc.robot;

/* IMPORTS */

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/*
 * Main class for initializing the robot
 * Handles status, like teleop, autonomous, and test
 * 
 */

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /*
   * Runs as soon as robot is powered on
   * initializes RobotContainer
   * 
   */

  public void robotInit() {
    m_robotContainer = new RobotContainer();

  }

  /*
   * Method that is run every 20 ms; during operation and when disabled
   * Runs command scheduler which organized all commands sent to the robot
   * Ensures that only one instance of a command per subsystem is run at a time
   * 
   */
  
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  // Runs whenever robot is disabled
  public void disabledInit() {}

  public void disabledPeriodic() {}

  // Runs autonomous command selected in RobotContainer
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // Autonomous command scheduler (placeholder)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  // Runs whenever robot is enabled in autonomous
  public void autonomousPeriodic() {}

  // Stops any autonomous functions as soon as teleop is granted
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  // Runs whenever robot is enabled in teleop
  public void teleopPeriodic() {}

  // Whenver test mode is enabled, stops all other commands
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  // Runs whenver robot is enabled in test mode
  public void testPeriodic() {}
}