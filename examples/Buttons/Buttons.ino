#include <ArduboyAdvance.h>

ArduboyAdvance ardu;

void setup() {
    ardu.begin(); //Initialize ArduboyAdvance
    ardu.setFrameRate(15); //Set framerate to 15 to save battery
}

void loop() {
    if (!(arduboy.nextFrame())) //Wait for the start of the next frame
        return;

    ardu.setCursorPos(1,1); //Start writing at pixel 1,1

    if (ardu.pressed(A_BUTTON)) { //Check if the A button is pressed
        ardu.print("A Button was pressed");
    } else if (ardu.pressed(B_BUTTON)) { //Check if the B button is pressed
        ardu.print("B Button was pressed");
    } else if (ardu.pressed(X_BUTTON)) { //Check if the X button is pressed
        ardu.print("X Button was pressed");
    } else if (ardu.pressed(Y_BUTTON)) { //Check if the Y button is pressed
        ardu.print("Y Button was pressed");
    } else if (ardu.pressed(SEL_BUTTON)) { //Check if the Select button is pressed(Press the joystick)
        ardu.print("Select Button was pressed");
    }

    ardu.display(CLEAR_BUFFER); //Display the contents of the screen buffer and clear it 
}