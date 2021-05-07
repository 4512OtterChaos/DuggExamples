// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

    private WPI_TalonSRX leftA = new WPI_TalonSRX(3); // left lead
    private WPI_TalonSRX leftB = new WPI_TalonSRX(4);
    private WPI_TalonSRX rightA = new WPI_TalonSRX(2); // right lead
    private WPI_TalonSRX rightB = new WPI_TalonSRX(1);

    private Encoder leftEncoder = new Encoder(4, 5);
    private Encoder rightEncoder = new Encoder(2, 3);

    private ADXRS450_Gyro gyro = new ADXRS450_Gyro(Port.kOnboardCS0);

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
        log();
    }

    public void tankDriveVolts(double leftVolts, double rightVolts){
        SmartDashboard.putNumber("Left DT Volts", leftVolts);
        SmartDashboard.putNumber("Right DT Volts", rightVolts);
        leftA.setVoltage(leftVolts);
        rightA.setVoltage(rightVolts);
    }
    public void tankDrive(double left, double right){
        tankDriveVolts(left*12, right*12);
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public void resetEncoders(){
        leftEncoder.reset();
        rightEncoder.reset();
    }
    public void resetGyro(){
        gyro.reset();
    }

    public void log(){
        SmartDashboard.putNumber("Left Encoder", leftEncoder.get());
        SmartDashboard.putNumber("Right Encoder", rightEncoder.get());
        SmartDashboard.putNumber("Gyro Heading", getHeading().getDegrees());
    }
}
