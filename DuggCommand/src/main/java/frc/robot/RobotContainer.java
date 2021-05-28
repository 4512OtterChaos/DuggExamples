// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

    private Drivetrain drivetrain;
    private Elevator elevator;
    private Intake intake;

    private OCXboxController driver = new OCXboxController(0);
    
    public RobotContainer() {
        drivetrain = new Drivetrain();
        elevator = new Elevator();
        intake = new Intake();

        configureButtonBindings();
    }
    
    private void configureButtonBindings() {
        RunCommand teleDrive = new RunCommand(()->{
            drivetrain.tankDrive(driver.getLeftArcade(), driver.getRightArcade());
        }, drivetrain);

        drivetrain.setDefaultCommand(teleDrive);

        /*
        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->driver.setDriveSpeed(OCXboxController.kSpeedFast))
            .whenReleased(()->driver.setDriveSpeed(OCXboxController.kSpeedDefault));

        new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->driver.setDriveSpeed(OCXboxController.kSpeedMax))
            .whenReleased(()->driver.setDriveSpeed(OCXboxController.kSpeedDefault));
        */

        driver.bumperRightButton
            .whenPressed(()->elevator.setPercent(0.2),elevator)
            .whenReleased(()->elevator.setPercent(0),elevator);
        driver.bumperLeftButton
            .whenPressed(()->elevator.setPercent(-0.2),elevator)
            .whenReleased(()->elevator.setPercent(0),elevator);

        driver.aButton
            .whenPressed(()->elevator.setPosition(0));
        driver.bButton
            .whenPressed(()->elevator.setPosition(20));
        driver.yButton
            .whenPressed(()->elevator.setPosition(40));

        driver.triggerRightButton
            .whileHeld(()->intake.setIntake(driver.getTriggerAxis(Hand.kRight)),intake)
            .whenReleased(()->intake.setIntake(intake.kNominalVolts),intake);
        driver.triggerLeftButton
            .whileHeld(()->intake.setIntake(-driver.getTriggerAxis(Hand.kLeft)),intake)
            .whenReleased(()->intake.setIntake(intake.kNominalVolts),intake);
    }
    
    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
