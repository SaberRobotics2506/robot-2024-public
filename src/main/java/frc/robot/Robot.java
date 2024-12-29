// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  
  private Command m_autoSelected;
  private  SendableChooser<Command> auto_chooser;

  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  NetworkTable table = inst.getTable("auto");

  public StringPublisher currentAutoCommand;

  @Override
  public void robotInit() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
    m_robotContainer = new RobotContainer();
    auto_chooser = AutoBuilder.buildAutoChooser("Do Nothing");
    SmartDashboard.putData("Auto Choices", auto_chooser);
    
    currentAutoCommand = table.getStringTopic("currentAutoCommand").publish();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    currentAutoCommand.set(auto_chooser.getSelected().getName());
  }

  @Override
  public void disabledInit() {
        // m_led.stop();
  }

  @Override
  public void disabledPeriodic() {
    // m_robotContainer.leds.partyTime();
    // m_led.stop();
  }

  @Override
  public void disabledExit() {
    // m_led.stop();
  }

  @Override
  public void autonomousInit() {
    GlobalVariables.limelightTolerance = 100000000;
    m_autoSelected = auto_chooser.getSelected();
    System.out.println("Auto selected: " + m_autoSelected.getName());
    m_autoSelected.schedule();
  } 

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    GlobalVariables.limelightTolerance = Constants.aprilTagLeniancy;
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {
    // m_led.stop();
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

