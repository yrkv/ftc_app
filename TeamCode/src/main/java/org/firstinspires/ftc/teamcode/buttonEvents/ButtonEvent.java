package org.firstinspires.ftc.teamcode.buttonEvents;

/**
 * Used to handle and set up events with buttons on the gamepad, such as toggling a state.
 * Before we had this system, we needed to use instance fields in whichever opMode when we
 * wanted to have something happen when you press a button. Our system lets us assign some
 * button events to each gamepad at the start of the program, making the code much clearer
 * {@link ButtonEventManager#run()} should be called every iteration of the main loop.
 * @see ButtonEventManager#run()
 * @see Button
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
