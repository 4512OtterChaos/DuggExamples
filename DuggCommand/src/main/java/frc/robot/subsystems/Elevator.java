// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevConstants;
import frc.robot.common.MathHelp;

public class Elevator extends SubsystemBase {
    
    private VictorSP elevA = new VictorSP(0);
    private VictorSP elevB = new VictorSP(1);

    private Encoder encoder = new Encoder(6, 7);

    private DigitalInput switchUp = new DigitalInput(1);
    private DigitalInput switchDown = new DigitalInput(8);

    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);

    private double targetVolts = 0;
    private double targetInches = 0;
    private boolean isManual = true;

    private final double kAntiGrav = 0.9;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0.3, 0);
    private ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0, new Constraints(25, 30));

    public Elevator() {
        encoder.setDistancePerPulse(0.0191860465); // inches per edge

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }
    
    @Override
    public void periodic() {
        
        double adjustedVolts = targetVolts;

        if(!isManual){
            adjustedVolts = controller.calculate(getPosInches(), targetInches);
            adjustedVolts += feedforward.calculate(controller.getSetpoint().velocity);
        }

        adjustedVolts += kAntiGrav;

        // Bound elevator travel by limit switches and encoder counts
        if(getBotSwitch() || getPosInches() < -2){
            encoder.reset();
            adjustedVolts = Math.max(0, adjustedVolts);
        }
        if(getTopSwitch() || getPosInches() > 82) adjustedVolts = Math.min(adjustedVolts, kAntiGrav);

        SmartDashboard.putNumber("Elevator Volts", adjustedVolts);
        elevA.setVoltage(adjustedVolts);
        elevB.setVoltage(adjustedVolts);

        SmartDashboard.putNumber("Elevator Counts", getCounts());
        SmartDashboard.putNumber("Elevator Height", getPosInches());
        SmartDashboard.putNumber("Elevator Rate", getRateInches());

        double actualPercent = MathHelp.findPercentage(getPosInches(), ElevConstants.kLEDBottomPos, ElevConstants.kLEDTopPos);
        double setpoint = isManual ? getPosInches() : controller.getSetpoint().position;
        double setpointPercent = MathHelp.findPercentage(setpoint, ElevConstants.kLEDBottomPos, ElevConstants.kLEDTopPos);
        double goal = isManual ? getPosInches() : controller.getGoal().position;
        double goalPercent = MathHelp.findPercentage(goal, ElevConstants.kLEDBottomPos, ElevConstants.kLEDTopPos);

        int actualIndex = (int)MathHelp.lerp(actualPercent, 0, 59);
        int setpointIndex = (int)MathHelp.lerp(setpointPercent, 0, 59);
        int goalIndex = (int)MathHelp.lerp(goalPercent, 0, 59);

        for(int i=0;i<buffer.getLength();i++){
            if(i<=setpointIndex){
                if(i==setpointIndex) buffer.setHSV(i, 25, 255, 200);
                else buffer.setHSV(i, 30, 255, 50);
            }
            else buffer.setHSV(i, 0, 0, 0);
            if(i==goalIndex) buffer.setHSV(i, 120, 255, 200);
            if(i==actualIndex) buffer.setHSV(i, 0, 255, 200);
        }
        led.setData(buffer);
    }

    public void setPercent(double percent){
        setVolts(percent*12);
    }
    public void setVolts(double volts){
        isManual = true;
        targetVolts = volts;
    }

    public void setPosition(double inchesHeight){
        if(isManual) controller.reset(getPosInches(), getRateInches());
        isManual = false;
        targetInches = inchesHeight;
    }

    public int getCounts(){
        return encoder.get();
    }
    public double getPosInches(){
        return encoder.getDistance();
    }
    public double getRateInches(){
        return encoder.getRate();
    }

    public boolean getTopSwitch(){
        return switchUp.get();
    }
    public boolean getBotSwitch(){
        return switchDown.get();
    }

    public void resetLiftEncoder(){
        encoder.reset();
    }
}
