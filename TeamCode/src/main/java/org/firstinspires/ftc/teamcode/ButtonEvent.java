package org.firstinspires.ftc.teamcode;

/**
 * Used to handle and set up events with buttons on the gamepad.
 * {@link OpMode8696#runButtonEvents()} should be called every iteration of the main loop.
 *
 * @see OpMode8696#runButtonEvents()
 */

public abstract class ButtonEvent {
    /**
     * Which {@link Button} this implementation will handle events for.
     */
    public Button button;

    public ButtonEvent(Button button) {
        this.button = button;
    }

    /**
     * Called when the button is pressed down
     */
    public void onDown() {}

    /**
     * called when the button is released
     */
    public void onUp() {}

    /**
     * called repeatedly while the button is pressed down
     */
    public void whileDown() {}

    /**
     * called repeatedly while the button is pressed down
     */
    public void whileUp() {}
}
