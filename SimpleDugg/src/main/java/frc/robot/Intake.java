package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake {
    
    private Spark intakeR = new Spark(4);
    private Spark intakeL = new Spark(5);

    public Intake() {
        intakeR.setSafetyEnabled(false);
        intakeL.setSafetyEnabled(false);
        intakeR.setInverted(true);
    }

    public void setPercent(double percent) {
        intakeR.setVoltage(12*percent);
        intakeL.setVoltage(12*percent);
    }
}
