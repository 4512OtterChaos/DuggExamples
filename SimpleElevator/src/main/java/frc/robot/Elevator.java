package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

public class Elevator {

    private VictorSP elevA = new VictorSP(0);
    private VictorSP elevB = new VictorSP(1);

    // limit switches for top/bottom detection
    private DigitalInput switchUp = new DigitalInput(1);
    private DigitalInput switchDown = new DigitalInput(8);

    public Elevator() {
    }

    public void setPercent(double percent) {
        // use limit switches to check if the elevator should stop
        if(switchDown.get()) percent = Math.max(percent, 0);
        if(switchUp.get()) percent = Math.min(percent, 0);

        elevA.setVoltage(12*percent);
        elevB.setVoltage(12*percent);
    }
}
