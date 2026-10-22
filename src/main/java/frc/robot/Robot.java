// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Components.*;
import frc.robot.RobotStates.*;
import frc.robot.Scratchbot.SetUpLab;

public class Robot extends TimedRobot {
  public static Robot instance;
  public static XboxController controller = new XboxController(0);

  @Override
  public void robotInit() {
    //set up for position tracking 
    ComponentManager.initialize();
    PositionComponent.zeroPose();
    //initialize the instance variable
    instance = this;
  }

  @Override
  public void robotPeriodic() {
    //update position tracking
    ComponentManager.periodic();
    CommandScheduler.getInstance().run();
  }
  @Override
  public void teleopInit() {
    //controller control setup for teleop.
    RobotState.Initialize(controller);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    //Run Lab
    SetUpLab.initialize();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
