package frc.robot.utils;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.swerve.AutoBalancingPID;
import frc.robot.commands.swerve.SwerveTeleopCommand;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.Constants.Arm.ArmSetpoints;
import frc.robot.utils.Constants.VisionConstants.Piece;
import frc.robot.utils.Constants.VisionConstants.Pipelines;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 */
public class JoystickIO {
    private static String[] lastJoystickNames = new String[] { "", "", "", "", "", "" };

    private Xbox pilot;
    private Xbox operator;

    private Swerve swerve;

    public JoystickIO(Xbox pilot, Xbox operator) {
        this.pilot = pilot;
        this.operator = operator;
        this.swerve = Robot.swerve;
    }

    public static boolean didJoysticksChange() {
        boolean joysticksChanged = false;
        for (int port = 0; port < DriverStation.kJoystickPorts; port++) {
            String name = DriverStation.getJoystickName(port);
            if (!name.equals(lastJoystickNames[port])) {
                joysticksChanged = true;
                lastJoystickNames[port] = name;
            }
        }
        return joysticksChanged;
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
    public void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();

        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();

        } else {
            // if no joysticks are connected (ShuffleBoard buttons)
            noJoystickBindings();
        }

        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public void pilotAndOperatorBindings() {

        pilot.yWhileHeld(() -> swerve.toggleSpeed());

        pilot.rightTriggerWhileHeld(new AutoBalancingPID(swerve, 0));
        // swerve button bindings
        pilot.startWhileHeld(Commands.runOnce(() -> {
            swerve.setFlipGyro(false);
            swerve.resetGyro();
        }, swerve));

        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));
    }

    /**
     * If only one controller is plugged in (pi)
     */
    public void pilotBindings() {
        // arm setpoints (buttons)
        // pilot.bWhileHeld(new AlignApriltag(false));

        // arm setpoints (bumpers)
        // pilot.leftBumper().onTrue(new MoveArm(
        // ArmSetpoints.DOUBLE_SUBSTATION));

        // intake button bindings

        // pilot.leftTriggerWhileHeld(() ->
        // armIntake.setVoltage(Constants.ArmIntake.intakeVoltage),
        // () -> armIntake.setVoltage(Constants.ArmIntake.idleVoltage));

        // swerve button bindings// pilot.startWhileHeld(
        // () -> swerve.resetOdometry(new Pose2d(new Translation2d(0, 0), new
        // Rotation2d(swerve.getHeading()))),
        // swerve);
        // swerve rotation cardinals
        pilot.povUp().whileTrue(Commands.run(() -> swerve.turnToRotation(0)));
        pilot.povLeft().whileTrue(Commands.run(() -> swerve.turnToRotation(270)));
        pilot.povDown().whileTrue(Commands.run(() -> swerve.turnToRotation(180)));
        pilot.povRight().whileTrue(Commands.run(() -> swerve.turnToRotation(90)));
    }

    /**
     * If NO joysticks are plugged in (Buttons for commands are runnable in the
     * "Controls" tab in ShuffleBoard)
     */
    public void noJoystickBindings() {
        ShuffleboardTab controlsTab = Shuffleboard.getTab("Controls");

        ShuffleboardLayout armCommands = controlsTab
                .getLayout("Arm", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands


        // ArmSetpoints.DOUBLE_SUBSTATION));

        ShuffleboardLayout armIntakeCommands = controlsTab
                .getLayout("Arm Intake", BuiltInLayouts.kList)
                .withSize(2, 2)
                .withProperties(Map.of("Label position", "HIDDEN"));


    }

    /**
     * Sets the default commands
     */
    public void setDefaultCommands() {
        swerve.setDefaultCommand(new SwerveTeleopCommand(

                () -> -pilot.getLeftY(),
                () -> pilot.getLeftX(),
                () -> pilot.getRightX()));


    }
}
