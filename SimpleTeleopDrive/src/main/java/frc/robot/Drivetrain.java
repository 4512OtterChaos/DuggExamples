package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drivetrain {
    
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

    private void configDrive(WPI_TalonSRX... motors) {
        for(WPI_TalonSRX motor : motors){
            motor.configFactoryDefault();
            motor.setNeutralMode(NeutralMode.Brake);
            motor.configOpenloopRamp(0.08);
            motor.configVoltageCompSaturation(12);
            motor.enableVoltageCompensation(true);
        }
    }

    public void tankDrive(double leftPercent, double rightPercent) {
        leftA.set(leftPercent);
        rightA.set(rightPercent);
    }
    
    public void arcadeDrive(double forwardPercent, double turnPercent) {
        // translate into left/right drivetrain side speeds
        double leftPercent = forwardPercent - turnPercent;
        double rightPercent = forwardPercent + turnPercent;

        // re-scale percentages (we can only go 100% speed)
        double maxMagnitude = Math.max(Math.abs(leftPercent), Math.abs(rightPercent));
        if(maxMagnitude > 1.0) {
            leftPercent /= maxMagnitude;
            rightPercent /= maxMagnitude;
        }

        tankDrive(leftPercent, rightPercent);
    }
}
