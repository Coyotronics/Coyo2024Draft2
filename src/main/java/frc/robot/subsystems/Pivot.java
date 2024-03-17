// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Pivot extends SubsystemBase
{
    //Declare our motors here first
    CANSparkMax pivotMotor1 = new CANSparkMax(35,MotorType.kBrushless);
    CANSparkMax pivotMotor2 = new CANSparkMax(36, MotorType.kBrushless);
        
    private Encoder throughBore = new Encoder(1,2,3);

    /*
    //fixed angles 
    private static final double horizontalAngle = x;
    private static final double angledSubwooferAngle = x;
    private static final double shallowerAngle = x; 
    private static final double verticalAngle = x;
    */
    public void pivot(boolean xButtonPressed, boolean yButtonPressed, boolean getXButtonReleased, boolean getYButtonReleased, boolean getRightBumper)
    {
        SmartDashboard.putNumber("Encoder Current Angle", throughBore.getDistance());
        if(xButtonPressed)
        {
            double currentAngle =  throughBore.getDistance();
            double expectedAngle = throughBore.getDistance()+15;
            pivotUp(currentAngle, expectedAngle);
        }

        else if (getXButtonReleased)
        {
            stopPivot();
        }
        else if(yButtonPressed)
        {
            //Test Angles
            //0 -15
            //-15 -30
            
            double currentAngle = throughBore.getDistance();
            double expectedAngle = throughBore.getDistance()-15;
            pivotDown(currentAngle, expectedAngle);
            
        }
        else if (getYButtonReleased)
        {
            stopPivot();
        }
        else if (getRightBumper)
        {
            stopPivot();
        }
        
    }

    public void stopPivot()
    {
        pivotMotor1.stopMotor();
        pivotMotor2.stopMotor();
    }

    public void pivotUp(double currentAngle,double expectedAngle)
    {
        SmartDashboard.putNumber("Expected Voltage",(double)((expectedAngle-currentAngle)/15.0)*2);
        if(currentAngle >= expectedAngle)
        {
            return;
        }
        else
        {
            pivotMotor2.follow(pivotMotor1,true);
            //Test Angle 1:
            //((-15-0)/15)*2 = -1*2 = -2
            //((-30+15)/15)*2= -1*2 = -2
            pivotMotor1.setVoltage((double)((expectedAngle-currentAngle)/15.0)*2); //change the number multiplied based on requirement
            pivotUp(throughBore.getDistance(),expectedAngle);
        }
    }

    public void pivotDown(double currentAngle,double expectedAngle)
    {
        //Test Angle 1
        //(-15-0/15*2)*-1
        if(currentAngle <= expectedAngle)
        {
            return;
        }
        else
        {
            pivotMotor2.follow(pivotMotor1,true);
            pivotMotor1.setVoltage(((double)((currentAngle-expectedAngle)/15.0)*2)*-1);  //change the number multiplied based on requirement (both for this case)
            pivotDown(throughBore.getDistance(),expectedAngle);
        }
    }
}

/*

// old working code 


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

*/