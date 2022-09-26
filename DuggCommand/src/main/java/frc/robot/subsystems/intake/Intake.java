// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

    private Spark intakeR = new Spark(4);
    private Spark intakeL = new Spark(5);

    private double targetVolts = 0;

    public final double kNominalVolts = 0.2;

    public Intake() {
        intakeR.setSafetyEnabled(false);
        intakeL.setSafetyEnabled(false);
        intakeR.setInverted(true);
    }
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Volts", targetVolts);
        intakeR.setVoltage(targetVolts);
        intakeL.setVoltage(targetVolts);
    }
    
    public void setIntake(double percent){
        targetVolts = percent*12;
    }
    public void setIntakeVolts(double volts){
        targetVolts = volts;
    }
}
