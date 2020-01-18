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
  SmartDashboard smartDashboard;

  double kV = MPAutoConstants.kV;
  double kA = MPAutoConstants.kA;
  double kH = MPAutoConstants.kH;
  double kP = MPAutoConstants.kP;

  public FollowPath(Drivetrain drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    String fileName = "RtoL";
    try {
      MPTrajectory trajectory = new MPTrajectory(fileName);
      leftTrajectory = trajectory.leftTrajectory;
      rightTrajectory = trajectory.rightTrajectory;
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    
    if (!smartDashboard.getKeys().contains("kV")) {
      smartDashboard.putNumber("kV", kV);
    }

    if (!smartDashboard.getKeys().contains("kA")) {
      smartDashboard.putNumber("kA", kA);
    }
    if (!smartDashboard.getKeys().contains("kP")) {
      smartDashboard.putNumber("kP", kP);
    }
    if (!smartDashboard.getKeys().contains("kH")) {
      smartDashboard.putNumber("kH", kH);
    }
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.zeroGyro();
    startTime = Timer.getFPGATimestamp();
    // SmartDashboard.putNumber("kV", value)
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    currentTime = Timer.getFPGATimestamp();
    int trajIndex = (int)((currentTime - startTime)/0.02);
    double desiredHead = leftTrajectory.get(trajIndex).heading;
    double desiredLeftDistance = leftTrajectory.get(trajIndex).position;
    double desiredRightDistance = rightTrajectory.get(trajIndex).position;
    double desiredLeftAcc = leftTrajectory.get(trajIndex).acceleration;
    double desiredRightAcc = rightTrajectory.get(trajIndex).acceleration;
    double desiredLeftVel = leftTrajectory.get(trajIndex).velocity;
    double desiredRightVel = rightTrajectory.get(trajIndex).velocity;

    double currentLeftDistance = drivetrain.getLeftEncoder();
    double currentRightDistance = drivetrain.getRightEncoder();
    double currentHeading = drivetrain.getGyroYaw();

    double leftOutput = kV * desiredLeftVel 
    + kA * desiredRightVel
    + kP * (desiredLeftDistance - currentLeftDistance)
    + kH * (desiredHead - currentHeading);

    double rightOutput = kV * desiredRightVel
    + kA * desiredRightVel
    + kP * (desiredRightDistance - currentRightDistance)
    + kH * (desiredHead - currentHeading);

    drivetrain.drive(leftOutput, rightOutput);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
