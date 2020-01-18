/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Add your docs here.
 */
public class MPTrajectory {

  public List<MPPoint> leftTrajectory = new ArrayList<>();
  public List<MPPoint> rightTrajectory = new ArrayList<>();

  public MPTrajectory(String fileName) throws IOException {
    this(new File(fileName + "_left"), new File(fileName + "_right"));
  }

  public MPTrajectory(File leftFile, File rightFile) throws IOException {
    BufferedReader leftReader = new BufferedReader(new FileReader(leftFile));
    BufferedReader rightReader = new BufferedReader(new FileReader(rightFile));
    String[] line;
    while ((line = leftReader.readLine().split(",")) != null) {
      double pos = Double.parseDouble(line[0]);
      double vel = Double.parseDouble(line[1]);
      double acc = Double.parseDouble(line[2]);
      double hea = Double.parseDouble(line[3]);
      leftTrajectory.add(new MPPoint(pos, vel, acc, hea));
    }

    while ((line = rightReader.readLine().split(",")) != null) {
        double pos = Double.parseDouble(line[0]);
        double vel = Double.parseDouble(line[1]);
        double acc = Double.parseDouble(line[2]);
        double hea = Double.parseDouble(line[3]);
        rightTrajectory.add(new MPPoint(pos, vel, acc, hea));
    }

    leftReader.close();
    rightReader.close();
  }

}