/*
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


//ACTUAL WORKING CODE FROM DINA's COMPUTER


package frc.robot.subsystems;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class ShooterTest1 extends SubsystemBase
{
    //Declare our motors here first
    CANSparkMax pivotMotor1 = new CANSparkMax(35,MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(36, MotorType.kBrushless);
    CANSparkMax shooter1 = new CANSparkMax(37,MotorType.kBrushless);
    CANSparkMax shooter2 = new CANSparkMax(38,MotorType.kBrushless);
    CANSparkMax intake = new CANSparkMax(39,MotorType.kBrushless);
    


public void shoot(boolean buttonPressed,boolean Button1,boolean Button2,boolean yButton,boolean stopPivot,boolean stopIntake)
{
    SmartDashboard.putBoolean("Shooter Button Pressed",buttonPressed);
    SmartDashboard.putBoolean("Pivot Up Pressed",Button1);
    SmartDashboard.putBoolean("Pivot Down Pressed",Button2);
    SmartDashboard.putBoolean("Shooter Intake Active",yButton);
    intake.setInverted(true);

    if(stopPivot)
    {
        stopPivot();
    }

    if(stopIntake)
    {
        stopShooter();
    }
    //Checks if the Shooter Button was ever pressed
    if(buttonPressed)
    {
       shooter();
    }

    //Checks if the pivot up button was pressed
    if(Button1)
    {
        pivot(true, false);
    }
    //Checks if the pivot dwown button was pressed
    else if(Button2)
    {
        pivot(false, true);
    }

    if(yButton)
    {
        intakeShooter();
    }
}
public void stopPivot()
{
    pivotMotor1.stopMotor();
    pivotMotor2.stopMotor();
}

public void stopShooter()
{
    shooter1.stopMotor();
    shooter2.stopMotor();
}

public void stopIntake()
{
    intake.stopMotor();
}

public void pivot(boolean upPivot, boolean downPivot)
{
     pivotMotor2.follow(pivotMotor1,true);
     if(upPivot)
     {
         SmartDashboard.putBoolean("Voltage ",true);
         pivotMotor1.setVoltage(-4.5);
     }
     else if (downPivot) 
     {
           
         SmartDashboard.putBoolean("Voltage ",true);
         pivotMotor1.setVoltage(4.5);
     }

}



public void shooter()
{

    shooter2.follow(shooter1, true);
    shooter1.setVoltage(11.5);
    //shooter1.setInverted(!shooter2.getInverted());

}


public void intakeShooter()
{

    //intake.setInverted(true);
    intake.set(2016); //direction is good!
}
}

*/