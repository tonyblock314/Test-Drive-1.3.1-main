// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;



/** Add your docs here. */
public class Winch extends SubsystemBase {

  WPI_VictorSPX Winch = new WPI_VictorSPX(Constants.TOP_WINCH_1_CAN_ID);

  public Winch() {
    Winch.setInverted(true);
  }

   // Pushes new speed to intake wheel motor
   public void setWheelSpeed(double speed) {
    // Updates shooter wheel speed on dashboard
    SmartDashboard.putNumber("Shooter Speed: ", speed);
    System.out.println("Shooter Speed: " + speed);
    Winch.set(speed);
  }
}
