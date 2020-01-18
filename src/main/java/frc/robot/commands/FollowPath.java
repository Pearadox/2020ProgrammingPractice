/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.io.IOException;
import java.util.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.autonomous.MPPoint;
import frc.robot.autonomous.MPTrajectory;
import frc.robot.Constants.MPAutoConstants;
import edu.wpi.first.wpilibj.smartdashboard.*;

public class FollowPath extends CommandBase {
  /**
   * Creates a new FollowPath.
   */
  List<MPPoint> leftTrajectory;
  List<MPPoint> rightTrajectory;

  double currentTime;
  double startTime;
  
  Drivetrain drivetrain;
  int trajIndex;

  double kV = MPAutoConstants.kV;
  double kA = MPAutoConstants.kA;
  double kH = MPAutoConstants.kH;
  double kP = MPAutoConstants.kP;

  public FollowPath(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    String fileName = "sCurve";
    try {
      MPTrajectory trajectory = new MPTrajectory(fileName);
      leftTrajectory = trajectory.leftTrajectory;
      rightTrajectory = trajectory.rightTrajectory;
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    if (!SmartDashboard.getKeys().contains("kV")) {
      SmartDashboard.putNumber("kV", kV);
    }

    if (!SmartDashboard.getKeys().contains("kA")) {
      SmartDashboard.putNumber("kA", kA);
    }
    if (!SmartDashboard.getKeys().contains("kP")) {
      SmartDashboard.putNumber("kP", kP);
    }
    if (!SmartDashboard.getKeys().contains("kH")) {
      SmartDashboard.putNumber("kH", kH);
    }
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroGyro();
    startTime = Timer.getFPGATimestamp();
    kV = SmartDashboard.getNumber("kV", kV);
    kA = SmartDashboard.getNumber("kA", kA);
    kP = SmartDashboard.getNumber("kP", kP);
    kH = SmartDashboard.getNumber("kH", kH);
    drivetrain.zeroEncoders();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    // increment the iterator
    trajIndex = (int) Math.round((currentTime - startTime)/0.02);
    if (trajIndex > leftTrajectory.size()) { return; }
    //desired values
    double desiredHead = leftTrajectory.get(trajIndex).heading;
    double desiredLeftDistance = leftTrajectory.get(trajIndex).position;
    double desiredRightDistance = rightTrajectory.get(trajIndex).position;
    // double desiredLeftAcc = leftTrajectory.get(trajIndex).acceleration;
    // double desiredRightAcc = rightTrajectory.get(trajIndex).acceleration;
    double desiredLeftVel = leftTrajectory.get(trajIndex).velocity;
    double desiredRightVel = rightTrajectory.get(trajIndex).velocity;

    //current values
    double currentLeftDistance = drivetrain.getLeftEncoder();
    double currentRightDistance = drivetrain.getRightEncoder();
    double currentHeading = drivetrain.getGyroYaw();

    //calculate outputs
    double leftOutput = kV * desiredLeftVel 
    + kA * desiredRightVel
    + kP * (desiredLeftDistance - currentLeftDistance)
    + kH * (desiredHead - currentHeading);

    double rightOutput = kV * desiredRightVel
    + kA * desiredRightVel
    + kP * (desiredRightDistance - currentRightDistance)
    + kH * (desiredHead - currentHeading);

    SmartDashboard.putNumber("leftOutput", leftOutput);
    SmartDashboard.putNumber("rightOutput", rightOutput);
    SmartDashboard.putNumber("pLeft", kP * (desiredLeftDistance - currentLeftDistance));
    SmartDashboard.putNumber("pRight", kP * (desiredRightDistance - currentRightDistance));
    SmartDashboard.putNumber("h", kH * (desiredHead - currentHeading));
    SmartDashboard.putNumber("leftEncoder", drivetrain.getLeftEncoder());
    SmartDashboard.putNumber("rightEncoder", drivetrain.getRightEncoder());
    //integrate outputs
    drivetrain.drive(leftOutput, rightOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (trajIndex >= leftTrajectory.size() - 1){
      return true;
    }
    return false;
  }
}
