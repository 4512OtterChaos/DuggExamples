// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
        Command teleDrive = new RunCommand(()->{
            WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(
                driver.getForward(),
                -driver.getTurn(),
                true
            );
            drivetrain.tankDrivePercent(wheelSpeeds.left, wheelSpeeds.right);
        }, drivetrain).beforeStarting(()->driver.resetLimiters());

        drivetrain.setDefaultCommand(teleDrive);

        /*
        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->driver.setDriveSpeed(OCXboxController.kSpeedFast))
            .whenReleased(()->driver.setDriveSpeed(OCXboxController.kSpeedDefault));

        new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->driver.setDriveSpeed(OCXboxController.kSpeedMax))
            .whenReleased(()->driver.setDriveSpeed(OCXboxController.kSpeedDefault));
        */

        driver.rightBumper
            .whenPressed(()->elevator.setPercent(0.25),elevator)
            .whenReleased(()->elevator.setPercent(0),elevator);
        driver.leftBumper
            .whenPressed(()->elevator.setPercent(-0.25),elevator)
            .whenReleased(()->elevator.setPercent(0),elevator);

        driver.aButton
            .whenPressed(()->elevator.setPosition(0));
        driver.bButton
            .whenPressed(()->elevator.setPosition(20));
        driver.yButton
            .whenPressed(()->elevator.setPosition(40));

        driver.rightTriggerButton
            .whileHeld(()->intake.setIntake(driver.getRightTriggerAxis()),intake)
            .whenReleased(()->intake.setIntake(intake.kNominalVolts),intake);
        driver.leftTriggerButton
            .whileHeld(()->intake.setIntake(-driver.getLeftTriggerAxis()),intake)
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
