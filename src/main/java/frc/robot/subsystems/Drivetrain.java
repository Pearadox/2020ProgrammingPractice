/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import static frc.robot.Constants.DrivetrainConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  private final TalonSRX leftMaster;
  private final TalonSRX rightMaster;

  private final VictorSPX leftSlave1;
  private final VictorSPX leftSlave2;
  private final VictorSPX rightSlave1;
  private final VictorSPX rightSlave2;

  private final Encoder leftEncoder;
  private final Encoder rightEncoder;

  /**
   * Creates a new Drivetrain.
   */
  public Drivetrain() {
    leftMaster = new TalonSRX(LEFT_MASTER_CAN_ID);
    rightMaster = new TalonSRX(RIGHT_MASTER_CAN_ID);

    leftSlave1 = new VictorSPX(LEFT_SLAVE1_CAN_ID);
    leftSlave2 = new VictorSPX(LEFT_SLAVE2_CAN_ID);
    rightSlave1 = new VictorSPX(RIGHT_SLAVE1_CAN_ID);
    rightSlave2 = new VictorSPX(RIGHT_SLAVE2_CAN_ID);

    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);
    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);

    leftEncoder = new Encoder(LEFT_ENCODER_A, LEFT_ENCODER_B);
    rightEncoder = new Encoder(RIGHT_ENCODER_A, RIGHT_ENCODER_B);
  }

  public void arcadeDrive(double throttle, double twist, boolean squaredInputs) {
    if (squaredInputs) {
      throttle *= Math.abs(throttle);
      twist *= Math.abs(twist);
    }

    double leftOutput = throttle + twist;
    double rightOutput = throttle - twist;
    
    if (leftOutput > 1 || leftOutput < 1) {
      leftOutput = Math.copySign(1, leftOutput);
    }
    if (rightOutput > 1 || rightOutput < 1) {
      rightOutput = Math.copySign(1, rightOutput);
    }

    leftMaster.set(ControlMode.PercentOutput, leftOutput);
    rightMaster.set(ControlMode.PercentOutput, rightOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
