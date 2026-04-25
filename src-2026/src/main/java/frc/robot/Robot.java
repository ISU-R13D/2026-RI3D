// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Robot In 3 Days at Iowa State
// All of RI3D@ISU's modifications to this file are licensed under MIT License



package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.ResourceBundle.Control;
import java.util.function.IntSupplier;

import com.fasterxml.jackson.databind.util.ClassUtil.Ctor;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
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
  private final SparkMax intake_actuator = new SparkMax(9, MotorType.kBrushless);

  SparkMaxConfig left_drive_back_config;
  SparkMaxConfig left_drive_front_config;
  SparkMaxConfig right_drive_back_config;
  SparkMaxConfig right_drive_front_config;
  SparkMaxConfig shooter_secondary_config;
  SparkMaxConfig intake_config;
  SparkMaxConfig intake_actuator_config;

  RelativeEncoder intake_actuator_encoder;
  RelativeEncoder climb_encoder;
  boolean climb_isUp = false;
  boolean climb_toggle = false;
  boolean intake_actuator_isUp = true;


  public Robot() {
    initMotorConfig();  
    //CameraServer.startAutomaticCapture();
    //encoders
    intake_actuator_encoder = intake_actuator.getEncoder();
    climb_encoder = climber.getEncoder();

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
    SmartDashboard.putNumber("intake_actuator position", intake_actuator_encoder.getPosition());
    SmartDashboard.putNumber("climb position", climb_encoder.getPosition());
    SmartDashboard.putBoolean("climb_isUp", climb_isUp);
    SmartDashboard.putBoolean("climb_toggle", climb_toggle);


    robot_drive.arcadeDrive(controller.getLeftY(), controller.getRightX());

    if(controller.getBButton()){
      intake.set(-1);
    }

    if(controller.getYButton()){
      intake.set(0);
    }
    
    //future encoder stuff

    if(controller.getStartButtonPressed()){
      intake_actuator_encoder.setPosition(0);
      climb_encoder.setPosition(0);
    }


    if(controller.getRightBumperButtonPressed()){
      intake_actuator_isUp = !intake_actuator_isUp;
    }

    if(intake_actuator_isUp && intake_actuator_encoder.getPosition() > 0){
      intake_actuator.set(-0.15);
    }
    if(!intake_actuator_isUp && intake_actuator_encoder.getPosition() < 6){
      intake_actuator.set(0.05);
    }
    //future encoder stuff
    // if(!intake_actuator_down && intake_actuator_encoder.getPosition() <= 0)
    //   intake_actuator.set(0);

    shooter.set(controller.getRightTriggerAxis());
    shooter_secondary.set(-controller.getRightTriggerAxis());

    if(climb_isUp && climb_toggle){
      if(climb_encoder.getPosition() <=0){
        climb_isUp = false;
        climb_toggle = false;
      }
    }

    if(!climb_isUp && climb_toggle){
      if(climb_encoder.getPosition() >= 60){
        climb_isUp = true;
        climb_toggle = false;
      }
    }

    if(controller.getXButtonPressed()){
      climb_toggle = !climb_toggle;
    }

    if(climb_toggle){
      double climbSpeed = 0;
      if(climb_isUp)
        climbSpeed = -.4;
      else
        climbSpeed = .4;

      climber.set(climbSpeed);

    }
    else
      climber.set(0);

  
  }

  //This has been broken into it's own function for organizational purposes
  private void initMotorConfig(){

    left_drive_back_config = new SparkMaxConfig();
    left_drive_front_config = new SparkMaxConfig();
    right_drive_back_config = new SparkMaxConfig();
    right_drive_front_config = new SparkMaxConfig();
    intake_actuator_config = new SparkMaxConfig();


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




    left_drive_back.configure(left_drive_back_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    left_drive_front.configure(left_drive_front_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_drive_back.configure(right_drive_back_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    right_drive_front.configure(right_drive_front_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Here we use ResetSafeParameters and PersistParameters because these settings should remain between power cycles.
    //If later you set a temporary mode that does not need to persist between power cycles these should be set to NoResetSafeParameters and NoPersistParameters

  }



}