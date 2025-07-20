package scoringcontroller;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;

/**
 * Handle input from a custom Arduino Leonardo controller connected to the Driver Station.
 *
 * File based on "XboxController" class provided in the WPILib
 */
public class CustomController extends GenericHID implements Sendable {
  /** Represents a digital button on the custom controller. */
  public enum Button {
    /** 1 button. */
    k1(1),
    /** 2 button. */
    k2(2),
    /** 3 button. */
    k3(3),
    /** 4 button. */
    k4(4),
    /** 5 button. */
    k5(5),
    /** 6 button. */
    k6(6),
    /** 7 button. */
    k7(7),
    /** 8 button. */
    k8(8),
    /** 9 button. */
    k9(9),
    /** 10 button. */
    k10(10),
    /** 11 button. */
    k11(11),
    /** 12 button. */
    k12(12),
    /** 16 button. */
    k16(16),
    /** 17 button. */
    k17(17),
    /** 18 button. */
    k18(18),
    /** 19 button. */
    k19(19);

    /** Button value. */
    public final int value;

    Button(int value) {
      this.value = value;
    }

    /**
     * Get the human-friendly name of the button, matching the relevant methods. This is done by
     * stripping the leading `k`, and appending `Button`.
     *
     * <p>Primarily used for automated unit tests.
     *
     * @return the human-friendly name of the button.
     */
    @Override
    public String toString() {
      // Remove leading `k`
      return this.name().substring(1) + "Button";
    }
  }

  /**
   * Construct an instance of a controller.
   *
   * @param port The port index on the Driver Station that the controller is plugged into (0-5).
   */
  public CustomController(final int port) {
    super(port);
    HAL.report(tResourceType.kResourceType_Controller, port + 1);
  }

  /**
   * Read the value of the 1 button on the controller.
   *
   * @return The state of the button.
   */
  public boolean get1Button() {
    return getRawButton(Button.k1.value);
  }

  /**
   * Whether the 1 button was pressed since the last check.
   *
   * @return Whether the button was pressed since the last check.
   */
  public boolean get1ButtonPressed() {
    return getRawButtonPressed(Button.k1.value);
  }

  /**
   * Whether the 1 button was released since the last check.
   *
   * @return Whether the button was released since the last check.
   */
  public boolean get1ButtonReleased() {
    return getRawButtonReleased(Button.k1.value);
  }

  /**
   * Constructs an event instance around the 1 button's digital signal.
   *
   * @param loop the event loop instance to attach the event to.
   * @return an event instance representing the A button's digital signal
   *     attached to the given loop.
   */
  public BooleanEvent loop1(EventLoop loop) {
    return button(Button.k1.value, loop);
  }

  // Repeat for buttons 2 to 12, and 16 to 19
  public boolean get2Button() {
    return getRawButton(Button.k2.value);
  }

  public boolean get2ButtonPressed() {
    return getRawButtonPressed(Button.k2.value);
  }

  public boolean get2ButtonReleased() {
    return getRawButtonReleased(Button.k2.value);
  }

  public BooleanEvent loop2(EventLoop loop) {
    return button(Button.k2.value, loop);
  }

  public boolean get3Button() {
    return getRawButton(Button.k3.value);
  }

  public boolean get3ButtonPressed() {
    return getRawButtonPressed(Button.k3.value);
  }

  public boolean get3ButtonReleased() {
    return getRawButtonReleased(Button.k3.value);
  }

  public BooleanEvent loop3(EventLoop loop) {
    return button(Button.k3.value, loop);
  }

  public boolean get4Button() {
    return getRawButton(Button.k4.value);
  }

  public boolean get4ButtonPressed() {
    return getRawButtonPressed(Button.k4.value);
  }

  public boolean get4ButtonReleased() {
    return getRawButtonReleased(Button.k4.value);
  }

  public BooleanEvent loop4(EventLoop loop) {
    return button(Button.k4.value, loop);
  }

  public boolean get5Button() {
    return getRawButton(Button.k5.value);
  }

  public boolean get5ButtonPressed() {
    return getRawButtonPressed(Button.k5.value);
  }

  public boolean get5ButtonReleased() {
    return getRawButtonReleased(Button.k5.value);
  }

  public BooleanEvent loop5(EventLoop loop) {
    return button(Button.k5.value, loop);
  }

  public boolean get6Button() {
    return getRawButton(Button.k6.value);
  }

  public boolean get6ButtonPressed() {
    return getRawButtonPressed(Button.k6.value);
  }

  public boolean get6ButtonReleased() {
    return getRawButtonReleased(Button.k6.value);
  }

  public BooleanEvent loop6(EventLoop loop) {
    return button(Button.k6.value, loop);
  }

  public boolean get7Button() {
    return getRawButton(Button.k7.value);
  }

  public boolean get7ButtonPressed() {
    return getRawButtonPressed(Button.k7.value);
  }

  public boolean get7ButtonReleased() {
    return getRawButtonReleased(Button.k7.value);
  }

  public BooleanEvent loop7(EventLoop loop) {
    return button(Button.k7.value, loop);
  }

  public boolean get8Button() {
    return getRawButton(Button.k8.value);
  }

  public boolean get8ButtonPressed() {
    return getRawButtonPressed(Button.k8.value);
  }

  public boolean get8ButtonReleased() {
    return getRawButtonReleased(Button.k8.value);
  }

  public BooleanEvent loop8(EventLoop loop) {
    return button(Button.k8.value, loop);
  }

  public boolean get9Button() {
    return getRawButton(Button.k9.value);
  }

  public boolean get9ButtonPressed() {
    return getRawButtonPressed(Button.k9.value);
  }

  public boolean get9ButtonReleased() {
    return getRawButtonReleased(Button.k9.value);
  }

  public BooleanEvent loop9(EventLoop loop) {
    return button(Button.k9.value, loop);
  }

  public boolean get10Button() {
    return getRawButton(Button.k10.value);
  }

  public boolean get10ButtonPressed() {
    return getRawButtonPressed(Button.k10.value);
  }

  public boolean get10ButtonReleased() {
    return getRawButtonReleased(Button.k10.value);
  }

  public BooleanEvent loop10(EventLoop loop) {
    return button(Button.k10.value, loop);
  }

  public boolean get11Button() {
    return getRawButton(Button.k11.value);
  }

  public boolean get11ButtonPressed() {
    return getRawButtonPressed(Button.k11.value);
  }

  public boolean get11ButtonReleased() {
    return getRawButtonReleased(Button.k11.value);
  }

  public BooleanEvent loop11(EventLoop loop) {
    return button(Button.k11.value, loop);
  }

  public boolean get12Button() {
    return getRawButton(Button.k12.value);
  }

  public boolean get12ButtonPressed() {
    return getRawButtonPressed(Button.k12.value);
  }

  public boolean get12ButtonReleased() {
    return getRawButtonReleased(Button.k12.value);
  }

  public BooleanEvent loop12(EventLoop loop) {
    return button(Button.k12.value, loop);
  }

  public boolean get16Button() {
    return getRawButton(Button.k16.value);
  }

  public boolean get16ButtonPressed() {
    return getRawButtonPressed(Button.k16.value);
  }

  public boolean get16ButtonReleased() {
    return getRawButtonReleased(Button.k16.value);
  }

  public BooleanEvent loop16(EventLoop loop) {
    return button(Button.k16.value, loop);
  }

  public boolean get17Button() {
    return getRawButton(Button.k17.value);
  }

  public boolean get17ButtonPressed() {
    return getRawButtonPressed(Button.k17.value);
  }

  public boolean get17ButtonReleased() {
    return getRawButtonReleased(Button.k17.value);
  }

  public BooleanEvent loop17(EventLoop loop) {
    return button(Button.k17.value, loop);
  }

  public boolean get18Button() {
    return getRawButton(Button.k18.value);
  }

  public boolean get18ButtonPressed() {
    return getRawButtonPressed(Button.k18.value);
  }

  public boolean get18ButtonReleased() {
    return getRawButtonReleased(Button.k18.value);
  }

  public BooleanEvent loop18(EventLoop loop) {
    return button(Button.k18.value, loop);
  }

  public boolean get19Button() {
    return getRawButton(Button.k19.value);
  }

  public boolean get19ButtonPressed() {
    return getRawButtonPressed(Button.k19.value);
  }

  public boolean get19ButtonReleased() {
    return getRawButtonReleased(Button.k19.value);
  }

  public BooleanEvent loop19(EventLoop loop) {
    return button(Button.k19.value, loop);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("HID");
    builder.publishConstString("ControllerType", "Custom Controller");
    builder.addBooleanProperty("1", this::get1Button, null);
    builder.addBooleanProperty("2", this::get2Button, null);
    builder.addBooleanProperty("3", this::get3Button, null);
    builder.addBooleanProperty("4", this::get4Button, null);
    builder.addBooleanProperty("5", this::get5Button, null);
    builder.addBooleanProperty("6", this::get6Button, null);
    builder.addBooleanProperty("7", this::get7Button, null);
    builder.addBooleanProperty("8", this::get8Button, null);
    builder.addBooleanProperty("9", this::get9Button, null);
    builder.addBooleanProperty("10", this::get10Button, null);
    builder.addBooleanProperty("11", this::get11Button, null);
    builder.addBooleanProperty("12", this::get12Button, null);
    builder.addBooleanProperty("16", this::get16Button, null);
    builder.addBooleanProperty("17", this::get17Button, null);
    builder.addBooleanProperty("18", this::get18Button, null);
    builder.addBooleanProperty("19", this::get19Button, null);
  }
}
