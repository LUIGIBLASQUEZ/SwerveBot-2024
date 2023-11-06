package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/*
 * Calls startRobot command
 * DO NOT MAKE ANY CHANGES TO THIS FILE
 * 
 */

public final class Main {
  private Main() {}

  /*
   * Main initialization function. Do not perform any initialization here.
   * if this classes name is changed, change parameter name accordingly
   * 
   */
  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
