package frc.robot.subsystems.multisubsystemcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.Arm.ArmPosition;
import frc.robot.subsystems.arm.ArmCommands;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;

public class MultiSubsystemCommands {
    public enum OverallPosition {
        Stow(ElevatorPosition.Stow, ArmPosition.Stow),
        Loading(ElevatorPosition.Stow, ArmPosition.Loading_Coral),
        L1(ElevatorPosition.L1, ArmPosition.Stow),
        L2(ElevatorPosition.L2, ArmPosition.Stow),
        L3(ElevatorPosition.L3, ArmPosition.Stow),
        L4(ElevatorPosition.L4, ArmPosition.Stow),
        L4_Score(ElevatorPosition.L4, ArmPosition.L4_Score);

        ElevatorPosition _elevatorSetpoint;
        ArmPosition _armSetpoint;

        OverallPosition(ElevatorPosition elevatorSetpoint, ArmPosition armSetpoint) {
            _elevatorSetpoint = elevatorSetpoint;
            _armSetpoint = armSetpoint;
        }

        ElevatorPosition getElevatorPosition() {
            return _elevatorSetpoint;
        }

        ArmPosition getArmPosition() {
            return _armSetpoint;
        }
    }

    public enum GamepieceMode {
        ALGAE,
        CORAL;
    }

    private Elevator _elevator;
    private Arm _arm;
    private ElevatorCommands _elevatorCommands;
    private ArmCommands _armCommands;

    public MultiSubsystemCommands(Elevator elevator, Arm arm, ElevatorCommands elevatorCommands,
            ArmCommands armCommands) {
        _elevator = elevator;
        _arm = arm;
        _elevatorCommands = elevatorCommands;
        _armCommands = armCommands;
    }

    public Command setOverallSetpoint(OverallPosition setpoint) {
        return _elevatorCommands.setElevatorSetpoint(setpoint.getElevatorPosition())
                .alongWith(_armCommands.setArmPosition(setpoint.getArmPosition()))
                .unless(() -> !canMoveToPos(_elevator.getCurrentPos(), setpoint.getElevatorPosition(),
                        _arm.getCurrentPos(), setpoint.getArmPosition()));
    }

    public Command setGamepieceMode() {
        return new InstantCommand();
    // Set up the command in here.
    }

    public Command waitForOverallMechanism() {
        return _elevatorCommands.waitUntilAtSetpoint()
                .alongWith(_armCommands.waitUntilAtSetpoint());
    }

    private Command score(OverallPosition setpoint) {
        if (setpoint == OverallPosition.L4) {
            return setOverallSetpoint(OverallPosition.L4_Score)
                    .andThen(waitForOverallMechanism())
                    .andThen(_armCommands.spit())
                    .andThen(setOverallSetpoint(OverallPosition.L4))
                    .andThen(waitForOverallMechanism());
        } else {
            return _armCommands.spit();
        }
    }

    public Command scoreGamepieceAtPosition(Supplier<OverallPosition> setpoint) {
        return scoreGamepieceAtPosition(setpoint.get());
    }

    public Command scoreGamepieceAtPosition(OverallPosition setpoint) {
        if (setpoint == OverallPosition.Stow || setpoint == OverallPosition.Loading
                || setpoint == OverallPosition.L4_Score) {
            throw new RuntimeException("scoreGamepieceAtPosition cannot run to stow,loading,or L4_score");
        }
        return setOverallSetpoint(setpoint)
                .andThen(waitForOverallMechanism())
                .andThen(score(setpoint))
                .andThen(setOverallSetpoint(OverallPosition.Stow));
    }

    public Command loadGamepiece() {
        return Commands.either(loadAlgae(), loadCoral(), () -> _arm.getCurrentMode() == GamepieceMode.ALGAE);
    }

    public Command loadCoral() {
        return setOverallSetpoint(OverallPosition.Loading)
                .andThen(waitForOverallMechanism())
                .andThen(_armCommands.intake())
                .andThen(setOverallSetpoint(OverallPosition.Stow))
                .andThen(_armCommands.moveGamepieceToLightSensor())
                .unless(() -> !canMoveToPos(_elevator.getCurrentPos(), ElevatorPosition.Stow,
                        _arm.getCurrentPos(), ArmPosition.Loading_Coral));

    }

    public Command loadAlgae() {
        return setOverallSetpoint(OverallPosition.Loading)
                .andThen(waitForOverallMechanism())
                .andThen(_armCommands.intake())
                .andThen(setOverallSetpoint(OverallPosition.Stow))
                .unless(() -> !canMoveToPos(_elevator.getCurrentPos(), ElevatorPosition.L2,
                _arm.getCurrentPos(), ArmPosition.Loading_Algae));
    }

    public boolean canMoveToPos(ElevatorPosition currentElevator, ElevatorPosition desiredElevator,
            ArmPosition currentArm, ArmPosition desiredArm) {
        boolean canMoveArm = false;
        boolean canMoveElevator = false;

        if (_arm.getCurrentMode() == GamepieceMode.CORAL) {
            switch (currentElevator) {
                case L1:
                case L2:
                case L3:
                    canMoveArm = (desiredArm != ArmPosition.L4_Score) && (desiredArm != ArmPosition.Loading_Coral);
                    break;
                case L4:
                    canMoveArm = (desiredArm != ArmPosition.Loading_Coral);
                    break;
                case Stow:
                    canMoveArm = (desiredArm != ArmPosition.L4_Score);
                    break;
                default:
                    canMoveArm = false;
                    break;
            }

            switch (desiredElevator) {
                case L1:
                case L2:
                case L3:
                    canMoveElevator = (currentArm != ArmPosition.L4_Score) && (currentArm != ArmPosition.Loading_Coral);
                    break;
                case L4:
                    canMoveElevator = (currentArm != ArmPosition.Loading_Coral);
                    break;
                case Stow:
                    canMoveElevator = (currentArm != ArmPosition.L4_Score);
                    break;
                default:
                    canMoveElevator = false;
                    break;
            }
        } else {
            switch (currentElevator) {
                case L1:
                case L4:
                    canMoveArm = false;
                    break;
                case L2:
                case L3:
                    canMoveArm = desiredArm != ArmPosition.Algae_Score;
                    break;
                case Stow:
                    canMoveArm = desiredArm != ArmPosition.Loading_Algae;
                    break;
                default:
                    canMoveArm = false;
                    break;
            }

            switch (desiredElevator) {
                case L1:
                case L4:
                    canMoveElevator = false;
                    break;
                case L2:
                case L3:
                    canMoveElevator = currentArm != ArmPosition.Algae_Score;
                    break;
                case Stow:
                    canMoveElevator = currentArm != ArmPosition.Loading_Algae;
                    break;
                default:
                    canMoveElevator = false;
                    break;
            }
        }
        System.out.println("Arm: " + canMoveArm + " Elevator: " + canMoveElevator);

        return canMoveArm && canMoveElevator;
    }

}