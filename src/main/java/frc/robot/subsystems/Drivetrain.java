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
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
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

  private final AHRS gyro;

  private final DifferentialDriveOdometry odometry;

  private double lastLeftEncoder;
  private double lastRightEncoder;
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

    leftEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);
    rightEncoder.setDistancePerPulse(DISTANCE_PER_PULSE);

    gyro = new AHRS(SPI.Port.kMXP);

    gyro.zeroYaw();
    odometry = new DifferentialDriveOdometry(new Rotation2d());
  }

  public void arcadeDrive(double throttle, double twist, boolean squaredInputs) {
    if (squaredInputs) {
      throttle *= Math.abs(throttle);
      twist *= Math.abs(twist);
    }

    if (Math.abs(throttle) <= DEADBAND) {
      throttle = 0.0;
    }
    if (Math.abs(twist) <= DEADBAND) {
      twist = 0.0;
    }

    double leftOutput = throttle + twist;
    double rightOutput = throttle - twist;
    
    if (Math.abs(leftOutput) > MAX_OUTPUT) {
      leftOutput = Math.copySign(1, leftOutput);
    }
    if (Math.abs(rightOutput) > MAX_OUTPUT) {
      rightOutput = Math.copySign(1, rightOutput);
    }

    leftMaster.set(ControlMode.PercentOutput, leftOutput);
    rightMaster.set(ControlMode.PercentOutput, rightOutput);
  }

  public void drive(double leftOutput, double rightOutput){
    leftMaster.set(ControlMode.PercentOutput, leftOutput);
    rightMaster.set(ControlMode.PercentOutput, rightOutput);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(Math.toRadians(gyro.getYaw()));
  }

  // Radians
  public double getGyroYaw(){
    return Math.toRadians(gyro.getYaw());
  }

  public void zeroGyro(){
    gyro.zeroYaw();
  }

  public double getLeftEncoder(){
    return leftEncoder.getDistance();
  }

  public double getRightEncoder(){
    return rightEncoder.getDistance();
  }

  // Velocity in Meters/Second
  public double getLVelocity(){
    return (leftEncoder.getDistance() - lastLeftEncoder)/0.02;
  }

  public double getRVelocity(){
    return (rightEncoder.getDistance() - lastRightEncoder)/0.02;
  }

  @Override
  public void periodic() {
    odometry.update(getAngle(), leftEncoder.getDistance(), rightEncoder.getDistance());
    lastLeftEncoder = leftEncoder.getDistance();
    lastRightEncoder = rightEncoder.getDistance();
  }
}
