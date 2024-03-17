// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Shoot extends SubsystemBase
{
    //Declare our motors here first
    CANSparkMax shooter1 = new CANSparkMax(37,MotorType.kBrushless);
    CANSparkMax shooter2 = new CANSparkMax(38,MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(39,MotorType.kBrushless);

public void shoot(boolean aButtonPressed, boolean aButtonReleased, boolean getLeftBumperPressed, boolean getLeftBumperReleased)
{
    if(aButtonPressed)
    {
        shooter2.follow(shooter1, true);
        shooter1.setVoltage(7.5);
        //shooter1.setInverted(!shooter2.getInverted());
    }
    if (getLeftBumperPressed)
    {
        intake.setInverted(true);
        intake.set(2016); //direction is good!
    }

    else if(getLeftBumperReleased)
    {
    
        intake.stopMotor();

    }
    else if (aButtonReleased)
    {
        stopShooter();
    }


}

public void stopShooter()
{
        shooter1.stopMotor();
        shooter2.stopMotor();
}


}
