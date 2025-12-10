package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator.ElevatorPosition;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.GamepieceMode;

public final class ElevatorCommands {
    private Elevator _elevator;

    public ElevatorCommands(Elevator elevator) {
        _elevator = elevator;
    }

    public Command runElevatorPID() {
        return Commands.run(() -> {
            _elevator.runMotorsWithPID();
        } );
    }

    public Command decrementElevatorPosition() {
        return new InstantCommand();
    }
  
    public Command incrementElevatorPosition(){
        return new InstantCommand(
           () -> {
            _elevator.incrementElevatorPosition();
    
           }, _elevator);
    } 

    public Command setElevatorSetpoint(ElevatorPosition setpoint) {
        return new InstantCommand(
            () -> {
            _elevator.setSetpoint(setpoint);
            }, _elevator);
    }

    public Command waitUntilAtSetpoint() {
        return new WaitUntilCommand(_elevator::isAtSetpoint);
    }
}

