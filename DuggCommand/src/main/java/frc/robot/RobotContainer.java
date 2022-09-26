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
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {

    // subsystems
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
    
    // bind commands to button inputs
    private void configureButtonBindings() {
        // "arcade" drivetrain control for tele-op
        Command teleDrive = new RunCommand(()->{
            WheelSpeeds wheelSpeeds = DifferentialDrive.arcadeDriveIK(
                driver.getForward(),
                -driver.getTurn(),
                false
            );
            drivetrain.tankDrivePercent(wheelSpeeds.left, wheelSpeeds.right);
        }, drivetrain).beforeStarting(()->driver.resetLimiters());

        drivetrain.setDefaultCommand(teleDrive);

        // manual elevator speed control
        driver.rightBumper
            .whenPressed(()->elevator.setPercent(0.25),elevator)
            .whenReleased(()->elevator.setPercent(0),elevator);
        driver.leftBumper
            .whenPressed(()->elevator.setPercent(-0.25),elevator)
            .whenReleased(()->elevator.setPercent(0),elevator);

        // automatic elevator position control
        driver.aButton
            .whenPressed(()->elevator.setPosition(0));
        driver.bButton
            .whenPressed(()->elevator.setPosition(20));
        driver.yButton
            .whenPressed(()->elevator.setPosition(40));

        // manual intake speed control
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
