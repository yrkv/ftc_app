package org.firstinspires.ftc.teamcode.buttonEvents;

import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * Manages all the button events. Should be initialized after init is pressed.
 * Call {@link #addButtonEvent(int, ButtonEvent)} during init to configure events,
 * and call {@link #run()} somewhere during your main loop.
 */

public class ButtonEventManager {
    private Gamepad gamepad1;
    private Gamepad gamepad2;

    private ButtonEvent[][] buttonEvents = new ButtonEvent[2][Button.values().length];

    /**
     * Stores the state of all the buttons on both gamepads last
     * iteration of the main loop. Used to handle the logic with
     * {@link #run(int)} to trigger events when they
     * should be.
     */
    private int[] wasPressed = new int[2];

    public ButtonEventManager(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * Add events for a specific button
     * @param gamepad which gamepad the button is on
     * @param event the event to be called
     */
    public void addButtonEvent(int gamepad, ButtonEvent event) {
        buttonEvents[gamepad-1][event.button.ordinal()] = event;
    }

    /**
     * Add multiple events for a gamepad
     * @param gamepad which gamepad the button is on
     * @param events the events to be called
     */
    public void addButtonEvents(int gamepad, ButtonEvent[] events) {
        for (ButtonEvent event : events)
            addButtonEvent(gamepad, event);
    }

    /**
     * Runs the button events for both gamepads
     */
    public void run() {
        run(1);
        run(2);
    }

    /**
     * Runs the button events for a specific gamepad. Uses bitwise operators
     * to check the state of every button all at once and run the respective
     * button events. This method should be run every iteration of the loop.
     * @param gamepad which gamepad to run button events for
     */
    private void run(int gamepad) {
        int currPressed = getButtonsPressed(gamepad);

        gamepad--;

        if (currPressed >= 0 && wasPressed[gamepad] >= 0) { // don't bother if no buttons pressed
            // use bitwise operators to check all buttons at the same time
            int onDown    =  currPressed & ~wasPressed[gamepad];
            int onUp      = ~currPressed &  wasPressed[gamepad];
            int whileDown =  currPressed &  wasPressed[gamepad];
            int whileUp   = ~currPressed & ~wasPressed[gamepad];
            for (int i = 0; i < 15; i++) {
                ButtonEvent event = buttonEvents[gamepad][i];
                if (event != null) {
                    // select specific bits and check if the related event should be called.
                    if ((onDown    & (1 << i)) > 0) event.onDown();
                    if ((onUp      & (1 << i)) > 0) event.onUp();
                    if ((whileDown & (1 << i)) > 0) event.whileDown();
                    if ((whileUp   & (1 << i)) > 0) event.whileUp();
                }
            }
        }

        wasPressed[gamepad] = currPressed;
    }

    /**
     * Get the button data as an int from one of the gamepads.
     *
     * @param gamepad which gamepad to access.
     */

    private int getButtonsPressed(int gamepad) {
        int currPressed = 0;

        try {
            byte[] arr = ((gamepad == 1) ? gamepad1 : gamepad2).toByteArray(); // select the right gamepad
            int len = arr.length;

            // extract the button data from the byte array
            currPressed += arr[len-3]; currPressed = currPressed << 8;
            currPressed += arr[len-2] & 0b11111111;

            // the left-most bit of the byte is counted as the "negative" part,
            // and the sign is maintained when it becomes an int.
            // " & 0b11111111" limits it to the 8 bits I want.
            // This isn't an issue with the first part because there are only 7 used bits.
        } catch (RobotCoreException e) {
            e.printStackTrace();
        }

        return currPressed;
    }
}
