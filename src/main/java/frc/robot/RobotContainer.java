package frc.robot;

import java.io.FileWriter;
import java.io.IOException;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.*;
import frc.robot.utils.*;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.AutoConstants;
import frc.robot.utils.Constants.Arm.ArmSetpoints;

public class RobotContainer {
    private final Xbox pilot = new Xbox(0);
    private final Xbox operator = new Xbox(1);


    private final JoystickIO joystickIO = new JoystickIO(pilot, operator);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        DriverStation.removeRefreshedDataEventHandle(44000);

        configureButtonBindings();
        configureAuto();
        

        DataLogManager.logNetworkTables(true);
        DriverStation.startDataLog(DataLogManager.getLog(), true);

        try {
            FileWriter writer = new FileWriter("data.csv", false);
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }

        RobotController.setBrownoutVoltage(7.0);

        ShuffleData.put("Swerve", AutoConstants.autoChooser);

    }

    /**
     * Set controller button bindings
     */
    public void configureButtonBindings() {

        // if (!JoystickIO.didJoysticksChange())
        // return;
        // CommandScheduler.getInstance().getActiveButtonLoop().clear();

        joystickIO.pilotAndOperatorBindings();
        //hi adrita was here
        joystickIO.setDefaultCommands();

    }

    /**
     * @return Autonomous Command
     */
    public Command getAutonomousCommand() {
        return new PrintCommand(null);
        // return AutoCommands.getPieceAlign();
        // return AutoConstants.autoChooser.getSelected();
    }

    /**
     * Set event maps for autonomous
     */
    public void configureAuto() {
        
    }
}