// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

/**
* The VM is configured to automatically run this class, and to call the functions corresponding to
* each mode, as described in the TimedRobot documentation. If you change the name of this class or
* the package after creating this project, you must also update the build.gradle file in the
* project.
*/
public class Robot extends TimedRobot {

    private Drivetrain drivetrain;
    private XboxController controller;

    private SlewRateLimiter forwardLimiter = new SlewRateLimiter(1.0 / 0.5); // 0.5 seconds to 100% output
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(1.0 / 0.5);

    private Timer timer = new Timer();

    /**
    * This function is run when the robot is first started up and should be used for any
    * initialization code.
    */
    @Override
    public void robotInit() {
        drivetrain = new Drivetrain();
        controller = new XboxController(0);
    }
    
    @Override
    public void robotPeriodic() {}
    
    @Override
    public void autonomousInit() {
        timer.reset();
        timer.start(); // (stopwatch)
    }
    
    @Override
    public void autonomousPeriodic() {

        // this style of autonomous control is "dead reckoning"
        // e.g. assuming we go forward x speed for y seconds, we should go x*y units forward

        if(timer.get() < 2){ // +20% forward for 2 seconds
            drivetrain.arcadeDrive(0.2, 0);
        }
        else if(timer.get() < 4){ // +20% turn for 2 seconds
            drivetrain.arcadeDrive(0, 0.2);
        }
        else if(timer.get() < 6){ // +20% forward for 2 seconds
            drivetrain.arcadeDrive(0.2, 0);
        }
        else { // stop
            timer.stop();
            drivetrain.arcadeDrive(0, 0);
        }
    }
    
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
    }
    
    @Override
    public void disabledInit() {
        drivetrain.tankDrive(0, 0);
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
