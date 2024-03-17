// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/*
public class Speaker extends SubsystemBase
{
    //Declare our motors here first
    CANSparkMax pivotMotor1 = new CANSparkMax(35,MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(36, MotorType.kBrushless);
    CANSparkMax shooter1 = new CANSparkMax(37,MotorType.kBrushless);
    CANSparkMax shooter2 = new CANSparkMax(38,MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(39,MotorType.kBrushless);
    Encoder throughBore = new Encoder(0,1);
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");


public void Speaker(boolean placeHolder)
{
    SpeakerAllignment
}

public void SpeakerAllignment()
{
    int distance = (kSpeakerHeight-kBotHeight)/Math.tan(Math.toRadians(kLimelightAngle));
    if(distance<kSpeakerShotMaxRadius&&distance>kSpeakerShotMinRadius)
    {
        //Send Green LEDs
        //Needs to begin adjusting pivot
        int expectedTurning = Math.atan(kSpeakerHeight/distance);
        int expectedTurningDegrees = Math.toDegrees(expectedTurning);
        pivoting(throughBore.getPosition(),expectedTurningDegrees);

    }
    else
    {
        //Send Red LEDs
    }
}

public void pivoting(int currentDegrees, int finalDegrees,int distance)
{
    pivotMotor2.follow(pivotMotor1);
    if(currentDegrees>finalDegrees)
    {
        int newVoltage = ((currentDegrees-finalDegrees)/distance)*3;
        pivotMotor1.setVoltage(newVoltage*-1);
    }
    else if(currentDegrees<finalDegrees)
    {
        int newVoltage = ((finalDegrees-currentDegrees)/distance)*3;
        pivotMotor1.setVoltage(newVoltage);
    }
    else(currentDegrees==finalDegrees)
    {
        return;
    }
}

}
*/