package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.HardwareConstants;
import frc.robot.HardwareConstants.CAN;
import frc.robot.subsystems.elevator.Elevator;

import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.led.Animation;

public class LEDs extends SubsystemBase {

    CANdle _candle = new CANdle(CAN.CANDLE_ID);
    static int _numLEDs = 60; //leds/height * currentheight 
    boolean _alreadyRunning = false;
    LEDAnimation _currentAnimation = LEDAnimation.None;

    Elevator _elevator;

    public enum LEDAnimation {
        None(null, null, 0),

        // RobotIdle(new LEDColor(0, 0, 255), null, 0),

        BlinkGreen(null, new StrobeAnimation(0, 255, 0, 1, 0.5, _numLEDs), 3),

        SolidGreen(new LEDColor(0, 255, 0), null, 0),

        BlinkDarkBlue(null, new StrobeAnimation(10, 0, 150, 1, 0.5, _numLEDs), 3),

        SolidDarkBlue(new LEDColor(10, 0, 150), null, 0),

        PartyMode(null, new RainbowAnimation(100, 1, _numLEDs), 3),

        Bounce(null, new LarsonAnimation(0, 255, 0), 3),

        SolidTeal(new LEDColor(0, 225, 174), null, 0),

        SolidCoral(new LEDColor(255, 80, 15), null, 0),

        SolidRed(new LEDColor(255, 0, 0), null, 0);

        LEDColor _color;
        Animation _animation;
        int _secondsToRun;

        LEDAnimation(LEDColor color, Animation animation, int secondsToRun) {
            _color = color;
            _animation = animation;
            _secondsToRun = secondsToRun;
        }

        public LEDColor getColor() {
            return _color;
        }

        public Animation getAnimation() {
            return _animation;
        }

        public int getSecondsToRun() {
            return _secondsToRun;
        }
    }

    public LEDs() {
        _candle.setLEDs(0, 0, 0);
        _candle.clearAnimation(0);
    }

    public void runAnimation(LEDAnimation animation) {
        if (animation != _currentAnimation) {
            _currentAnimation = animation;

            if (animation.getColor() == null) {
                _candle.clearAnimation(0);
                _candle.setLEDs(0, 0, 0);
                _candle.animate(animation.getAnimation());
            } else if (animation == LEDAnimation.SolidRed) {
                _candle.clearAnimation(0);
                LEDColor color = animation.getColor();
                _candle.setLEDs(color.getR(), color.getG(), color.getB(),0, 0, 
                (int) Math.round(_numLEDs/_elevator.getElevatorMaxHeight() * _elevator.getCurrentPosInches()));
            } else if (animation.getAnimation() == null) {
                LEDColor color = animation.getColor();
                _candle.clearAnimation(0);
                _candle.setLEDs(0, 0, 0);
                _candle.setLEDs(color.getR(), color.getG(), color.getB());
            } else {
                _candle.clearAnimation(0);
                _candle.animate(animation.getAnimation());
                LEDColor color = animation.getColor();
                _candle.setLEDs(color.getR(), color.getG(), color.getB());
            }

        }
    }

    public void intaking() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.BlinkGreen);
            _alreadyRunning = true;
        }
    }

    public void hasPiece() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.SolidGreen);
            _alreadyRunning = false;
        }
    }

    public void pickingUpCoral() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.SolidCoral);
            _alreadyRunning = false;
        }
    }

    public void elevatorOrArmIsMoving() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.SolidDarkBlue);
            _alreadyRunning = false;
        }
    }

    public void pickingUpAlgae() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.SolidTeal);
            _alreadyRunning = false;
        }
    }

    public void elevatorAndArmAtSetpoints() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.SolidDarkBlue);
            _alreadyRunning = false;
        }
    }

    public void robotHasClimbed() {
        if (!_alreadyRunning) {
            runAnimation(LEDAnimation.Bounce);
            _alreadyRunning = false;
        }
    }

    public void disabledAnimation1() {
    if (!_alreadyRunning) {
        runAnimation(LEDAnimation.PartyMode);
        _alreadyRunning = false;
    }
    }
    
    public void reset() {
        _candle.clearAnimation(0);
        _alreadyRunning = false;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LEDs/alreadyRunning", _alreadyRunning);
        Logger.recordOutput("LEDs/CurrentAnimation", _currentAnimation.toString());
    }
}
