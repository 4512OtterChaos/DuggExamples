// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MathHelp;

import static frc.robot.subsystems.elevator.ElevConstants.*;

public class Elevator extends SubsystemBase {
    
    private VictorSP elevA = new VictorSP(0);
    private VictorSP elevB = new VictorSP(1);

    private Encoder encoder = new Encoder(6, 7);

    // limit switches for top/bottom detection
    private DigitalInput switchUp = new DigitalInput(1);
    private DigitalInput switchDown = new DigitalInput(8);

    private AddressableLED led = new AddressableLED(9);
    private AddressableLEDBuffer buffer = new AddressableLEDBuffer(60);

    private double targetVolts = 0;
    private double targetInches = 0;
    private boolean isManual = true;

    // resist gravity
    private final double kAntiGrav = 1.1;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kStaticFF, kVelocityFF, kAccelFF);
    private ProfiledPIDController controller = new ProfiledPIDController(kP, kI, kD, kConstraintsInches);

    public Elevator() {
        encoder.setDistancePerPulse(0.0191860465); // inches per edge

        led.setLength(buffer.getLength());
        led.setData(buffer);
        led.start();
    }
    
    @Override
    public void periodic() {
        
        double adjustedVolts = targetVolts;

        // handoff between manual / automatic control
        if(!isManual){
            adjustedVolts = controller.calculate(getPosInches(), targetInches);
            adjustedVolts += feedforward.calculate(controller.getSetpoint().velocity);
        }

        // use anti-gravity
        if(!(getPosInches()<1 && targetVolts<=0)) adjustedVolts += kAntiGrav;

        // Limit elevator travel by limit switches and encoder counts
        if(getBotSwitch() || getPosInches() <= 0){
            encoder.reset();
            adjustedVolts = Math.max(0, adjustedVolts);
        }
        if(getTopSwitch() || getPosInches() > 82) adjustedVolts = Math.min(adjustedVolts, kAntiGrav);

        elevA.setVoltage(adjustedVolts);
        elevB.setVoltage(adjustedVolts);

        // log
        SmartDashboard.putNumber("Elevator Volts", adjustedVolts);
        SmartDashboard.putNumber("Elevator Counts", getCounts());
        SmartDashboard.putNumber("Elevator Height", getPosInches());
        SmartDashboard.putNumber("Elevator Rate", getRateInches());

        displayLED();
    }

    // manual speed control
    public void setPercent(double percent){
        setVolts(percent*12);
    }
    public void setVolts(double volts){
        isManual = true;
        targetVolts = volts;
    }

    // automatic position control
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

    // use LEDs to indicate elevator position/setpoint
    public void displayLED(){
        double actualPercent = MathHelp.findPercentage(getPosInches(), ElevConstants.kLEDBottomPos, ElevConstants.kLEDTopPos);
        double setpoint = isManual ? getPosInches() : controller.getSetpoint().position;
        double setpointPercent = MathHelp.findPercentage(setpoint, ElevConstants.kLEDBottomPos, ElevConstants.kLEDTopPos);
        double goal = isManual ? getPosInches() : controller.getGoal().position;
        double goalPercent = MathHelp.findPercentage(goal, ElevConstants.kLEDBottomPos, ElevConstants.kLEDTopPos);

        int actualIndex = (int)MathHelp.lerp(actualPercent, 0, 59);
        int setpointIndex = (int)MathHelp.lerp(setpointPercent, 0, 59);
        int goalIndex = (int)MathHelp.lerp(goalPercent, 0, 59);

        Color[] positionColors = new Color[3]; // actual, setpoint, goal

        for(int i=0;i<positionColors.length;i++){
            int index = 0;
            if(i==0) index = actualIndex;
            if(i==1) index = setpointIndex;
            if(i==2) index = goalIndex;
            Color old = positionColors[i];
            if(old==null) old = Color.kBlack;
            if(index==actualIndex) old = new Color(old.red+1,old.green,old.blue);
            if(index==setpointIndex) old = new Color(old.red,old.green,old.blue+1);
            if(index==goalIndex) old = new Color(old.red,old.green+1,old.blue);
            positionColors[i] = old;
        }

        for(int i=0;i<buffer.getLength();i++){
            if(i<=setpointIndex){
                if(i==setpointIndex) buffer.setLED(i, positionColors[1]);
                else buffer.setHSV(i, 25, 255, 50);
            }
            else buffer.setHSV(i, 0, 0, 0);
            if(i==goalIndex) buffer.setLED(i, positionColors[2]);
            if(i==actualIndex) buffer.setLED(i, positionColors[0]);
        }
        led.setData(buffer);
    }
}
