// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Class to run the algae rollers over CAN */
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armLeader;
  private final SparkMax armFollower;

  public ArmSubsystem() {
    // Set up the roller motors as brushed motors
    armLeader = new SparkMax(ArmConstants.ARM_LEADER_ID, MotorType.kBrushed);
    armFollower = new SparkMax(ArmConstants.ARM_FOLLOWER_ID, MotorType.kBrushed);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    armLeader.setCANTimeout(250);
    armFollower.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig armConfig = new SparkMaxConfig();
    armConfig.voltageCompensation(ArmConstants.ARM_MOTOR_VOLTAGE_COMP);
    armConfig.smartCurrentLimit(ArmConstants.ARM_MOTOR_CURRENT_LIMIT);
    armConfig.inverted(false);
    armConfig.idleMode(IdleMode.kBrake);

    armConfig.follow(armLeader);
    armFollower.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    armConfig.disableFollowerMode();
    armConfig.inverted(true);
    armLeader.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runArm(double forward, double reverse) {
    armLeader.set(forward - reverse);
  }
}
