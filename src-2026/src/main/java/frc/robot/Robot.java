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
  private final SparkMax  test_motor = new SparkMax(1, MotorType.kBrushless);

  SparkMaxConfig test_motor_config;


  public Robot() {
    initMotorConfig();  

  }

  @Override
  public void teleopPeriodic() {


  }

  //This has been broken into it's own function for organizational purposes
  private void initMotorConfig(){

    test_motor_config = new SparkMaxConfig();


    test_motor_config
    .inverted(true);

    test_motor.configure(test_motor_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Here we use ResetSafeParameters and PersistParameters because these settings should remain between power cycles.
    //If later you set a temporary mode that does not need to persist between power cycles these should be set to NoResetSafeParameters and NoPersistParameters

  }


}