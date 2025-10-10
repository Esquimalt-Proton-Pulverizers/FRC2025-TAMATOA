
package frc.robot.subsystems.lights;

import com.ctre.phoenix.led.*;
//import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer.RobotModes;


// https://github.com/FRC2539/javabot-2023/blob/main/src/main/java/frc/robot/subsystems/LightsSubsystem.java


public class LEDlights extends SubsystemBase {
   private final CANdle candle = new CANdle(26);
   //private final int LEDcount = 300;
   private CANdleConfiguration config;


   //private Animation m_toAnimate = null;


   /* public enum AnimationTypes {
       ColorFlow,
       Fire,
       Larson,
       Rainbow,
       RgbFade,
       SingleFade,
       Strobe,
       Twinkle,
       TwinkleOff,
       SetAll
   }


   private AnimationTypes m_currentAnimation;
   */

   public LEDlights() {
       config = new CANdleConfiguration();
       config.brightnessScalar = 1.0;
       config.disableWhenLOS = false;
       config.statusLedOffWhenActive = true;
       config.stripType = CANdle.LEDStripType.RGB;
       //config.vBatOutputMode = VBatOutputMode.Modulated;
       candle.configAllSettings(config,100);

       setColor(5,0,5); //Default colour setting
   }

   /* private void setColor(Colour colour) {
       candle.setLEDs(colour.r, colour.g, colour.b);
   } */
   private void setColor(int r,int g, int b) {
       candle.setLEDs(r, g, b);
   }


   /* private void setStrobe(Colour colour) {
       candle.animate(new StrobeAnimation(colour.r, colour.g, colour.b));
   }


   public Command setColorCommand(Colour colour) {
       return Commands.runOnce(() -> setColor(colour));
   }


   public Command setStrobeCommand(Colour colour) {
       return Commands.runOnce(() -> setStrobe(colour));
   }


   public Command clearCANdleCommand() {
       return setColorCommand(new Colour(0, 0, 0));
   } */


   public static class Colour {


       public int r;
       public int g;
       public int b;


       public Colour(int r, int g, int b) {
           this.r = r;
           this.g = g;
           this.b = b;
       }
   }

   public void lightMode(RobotModes robotMode) {
    if (robotMode == RobotModes.AlgaeMode) {
        setColor(0,255,0); //green
    }
    else if (robotMode == RobotModes.CoralMode) {
        setColor(255,255,255); //purple
    }
    else if (robotMode == RobotModes.HangingMode) {
        setColor(0,0,255); //blue
    }
    else if (robotMode == RobotModes.ManualMoveMode) {
        setColor(255,0,0); //red
    }
   }
   public void dimLights() {
       setColor(5,0,5); //off
   }
}