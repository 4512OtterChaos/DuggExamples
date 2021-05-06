// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private WPI_TalonSRX leftA = new WPI_TalonSRX(3); // left lead
    private WPI_TalonSRX leftB = new WPI_TalonSRX(4);
    private WPI_TalonSRX rightA = new WPI_TalonSRX(2); // right lead
    private WPI_TalonSRX rightB = new WPI_TalonSRX(1);

    public Drivetrain() {
        configDrive(leftA,leftB,rightA,rightB);

        leftA.setInverted(false);
        leftB.follow(leftA);
        leftB.setInverted(InvertType.FollowMaster);
        rightA.setInverted(true);
        rightB.follow(rightA);
        rightB.setInverted(InvertType.FollowMaster);
    }

    private void configDrive(WPI_TalonSRX... motors){
        for(WPI_TalonSRX motor : motors){
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configOpenloopRamp(DriveConstants.kRampRaw);
        }
    }

    @Override
    public void periodic() {
    }
}
