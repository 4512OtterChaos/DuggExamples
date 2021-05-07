// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lift extends SubsystemBase {
    
    private VictorSP liftA = new VictorSP(0);
    private VictorSP liftB = new VictorSP(1);

    private Encoder liftEncoder = new Encoder(6, 7);

    private DigitalInput switchUp = new DigitalInput(1);
    private DigitalInput switchDown = new DigitalInput(8);

    private double targetVolts = 0;

    public Lift() {
    }
    
    @Override
    public void periodic() {
        if(getBotSwitch()){
            liftEncoder.reset();
            targetVolts = Math.max(0, targetVolts);
        }
        if(getTopSwitch()) targetVolts = Math.min(targetVolts, 0);

        SmartDashboard.putNumber("Lift Volts", targetVolts);
        liftA.setVoltage(targetVolts);
        liftB.setVoltage(targetVolts);

        SmartDashboard.putNumber("Lift Pos", liftEncoder.get());
    }

    public void setLift(double percent){
        targetVolts = percent*12;
    }
    public void setLiftVolts(double volts){
        targetVolts = volts;
    }

    public int getLiftPos(){
        return liftEncoder.get();
    }

    public boolean getTopSwitch(){
        return switchUp.get();
    }
    public boolean getBotSwitch(){
        return switchDown.get();
    }

    public void resetLiftEncoder(){
        liftEncoder.reset();
    }
}
