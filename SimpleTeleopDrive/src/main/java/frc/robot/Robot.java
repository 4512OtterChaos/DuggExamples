// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    private XboxController controller;

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
    public void autonomousInit() {}
    
    @Override
    public void autonomousPeriodic() {}
    
    @Override
    public void teleopInit() {}
    
    @Override
    public void teleopPeriodic() {

        // "arcade" style joystick control
        double robotForward = -controller.getLeftY();
        double robotTurn = -controller.getRightX();

        // translate into left/right drivetrain side speeds
        double leftSpeed = robotForward - robotTurn;
        double rightSpeed = robotForward + robotTurn;

        // re-scale percentages (we can only go 100% speed)
        double maxMagnitude = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
        if(maxMagnitude > 1.0) {
            leftSpeed /= maxMagnitude;
            rightSpeed /= maxMagnitude;
        }

        // slow down drive speed for safety
        leftSpeed *= 0.4;
        rightSpeed *= 0.4;

        // set drivetrain motor speed
        drivetrain.drive(leftSpeed, rightSpeed);
    }
    
    @Override
    public void disabledInit() {
        drivetrain.drive(0, 0);
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
