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
import frc.robot.subsystems.Lift;

public class RobotContainer {

    private Drivetrain drivetrain;
    private Lift lift;
    private Intake intake;

    private OCXboxController driver = new OCXboxController(0);
    
    public RobotContainer() {
        drivetrain = new Drivetrain();
        lift = new Lift();
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

        new JoystickButton(driver, XboxController.Button.kBumperRight.value)
            .whenPressed(()->lift.setLift(0.5),lift)
            .whenReleased(()->lift.setLift(0),lift);
        new JoystickButton(driver, XboxController.Button.kBumperLeft.value)
            .whenPressed(()->lift.setLift(-0.5),lift)
            .whenReleased(()->lift.setLift(0),lift);

        double leftTrigger = driver.getTriggerAxis(Hand.kLeft);
        double rightTrigger = driver.getTriggerAxis(Hand.kRight);
        new Trigger(()->rightTrigger>0.2)
            .whenActive(()->intake.setIntake(rightTrigger),intake)
            .whenInactive(()->intake.setIntake(0),intake);
        new Trigger(()->leftTrigger>0.2)
            .whenActive(()->intake.setIntake(-leftTrigger),intake)
            .whenInactive(()->intake.setIntake(0),intake);
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
