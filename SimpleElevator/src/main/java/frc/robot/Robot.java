// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {

    private Drivetrain drivetrain;
    private Elevator elevator;

    private XboxController controller;
    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(1.0 / 0.5); // 0.5 seconds to 100% output
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(1.0 / 0.5);

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        drivetrain = new Drivetrain();
        elevator = new Elevator();

        controller = new XboxController(0);
    }
    
    @Override
    public void robotPeriodic() {}
    
    @Override
    public void autonomousInit() {}
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {
        forwardLimiter.reset(0);
        turnLimiter.reset(0);
    }
    
    @Override
    public void teleopPeriodic() {

        // "arcade" style joystick control
        double forwardPercent = -controller.getLeftY() * 0.4; // limit speed
        forwardPercent = forwardLimiter.calculate(forwardPercent); // limit acceleration
        double turnPercent = -controller.getRightX() * 0.4;
        turnPercent = turnLimiter.calculate(turnPercent);

        // set drivetrain motor speed
        drivetrain.arcadeDrive(forwardPercent, turnPercent);

        // manual elevator speed control
        if(controller.getRightBumper()){
            elevator.setPercent(0.25);
        }
        else if(controller.getLeftBumper()){
            elevator.setPercent(-0.25);
        }
        else {
            elevator.setPercent(0);
        }
    }
    
    @Override
    public void disabledInit() {
        drivetrain.tankDrive(0, 0);
        elevator.setPercent(0);
    }
    
    @Override
    public void disabledPeriodic() {}
    
    @Override
    public void testInit() {}
    
    @Override
    public void testPeriodic() {}
    
    @Override
    public void simulationInit() {}
    
    @Override
    public void simulationPeriodic() {}
}
