// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Robot In 3 Days at Iowa State
// All of RI3D@ISU's modifications to this file are licensed under MIT License



package frc.robot;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;


public class Robot extends TimedRobot {


  private final XboxController controller = new XboxController(0);
  private final DifferentialDrive robot_drive;

  private final SparkMax  left_drive_back = new SparkMax(1, MotorType.kBrushless);
  private final SparkMax  left_drive_front = new SparkMax(2, MotorType.kBrushless);
  private final SparkMax  right_drive_back = new SparkMax(3, MotorType.kBrushless);
  private final SparkMax  right_drive_front = new SparkMax(4, MotorType.kBrushless);
  private final SparkMax  shooter = new SparkMax(5, MotorType.kBrushless);
  private final SparkMax shooter_secondary = new SparkMax(6, MotorType.kBrushless);
  private final SparkMax intake = new SparkMax(7, MotorType.kBrushless);
  private final SparkMax climber = new SparkMax(8, MotorType.kBrushless);

  SparkMaxConfig left_drive_back_config;
  SparkMaxConfig left_drive_front_config;
  SparkMaxConfig right_drive_back_config;
  SparkMaxConfig right_drive_front_config;
  SparkMaxConfig shooter_secondary_config;
  SparkMaxConfig intake_config;

  public Robot() {
    initMotorConfig();  

    robot_drive = new DifferentialDrive(left_drive_back::set, right_drive_back::set);

    //Add the drive motors as a telemetry child to the robot drive 
    SendableRegistry.addChild(robot_drive, left_drive_front);
    SendableRegistry.addChild(robot_drive, left_drive_back);
    SendableRegistry.addChild(robot_drive, right_drive_front);
    SendableRegistry.addChild(robot_drive, right_drive_back);

    //Send the drivetrain telemetry data to the drivers station
    SmartDashboard.putData("Drivetrain", robot_drive);    


  }

  @Override
  public void teleopPeriodic() {
    robot_drive.arcadeDrive(controller.getLeftY(), controller.getRightX());

    if(controller.getBButton()){
      intake.set(.65);
    }

    if(controller.getYButton()){
      intake.set(0);
    }
    
    shooter.set(controller.getRightTriggerAxis());
    shooter_secondary.set(-controller.getRightTriggerAxis());

    if(controller.getXButton())
      climber.set(controller.getRawAxis(2));
    else
      climber.set(-controller.getRawAxis(2));

  }

  //This has been broken into it's own function for organizational purposes
  private void initMotorConfig(){

    left_drive_back_config = new SparkMaxConfig();
    left_drive_front_config = new SparkMaxConfig();
    right_drive_back_config = new SparkMaxConfig();
    right_drive_front_config = new SparkMaxConfig();
    intake_config = new SparkMaxConfig();

    left_drive_back_config
    .inverted(true)
    .openLoopRampRate(.5);

    left_drive_front_config
    .inverted(true)
    .follow(left_drive_back.getDeviceId())
    .openLoopRampRate(.5);

    right_drive_back_config
    .openLoopRampRate(.5);
    
    right_drive_front_config
    .follow(right_drive_back.getDeviceId())
    .openLoopRampRate(.5);

    intake_config
    .inverted(false);


    left_drive_back.configure(left_drive_back_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    left_drive_front.configure(left_drive_front_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_drive_back.configure(right_drive_back_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_drive_front.configure(right_drive_front_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    intake.configure(intake_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Here we use ResetSafeParameters and PersistParameters because these settings should remain between power cycles.
    //If later you set a temporary mode that does not need to persist between power cycles these should be set to NoResetSafeParameters and NoPersistParameters

  }


}