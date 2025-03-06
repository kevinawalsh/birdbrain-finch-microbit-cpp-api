#include "Finch.h"

void robot_main() {

    while (!Finch_getButton('T')) {
        char c = Finch_getOrientation();
        Finch_println(c);
    }
    
    for (int i = 0; i < 2; i++) {
        Finch_setBeak(255, 0, 255);
        Program_pause(0.1);
        Finch_setBeak(0, 255, 255);
        Program_pause(0.1);
        Finch_setBeak(255, 255, 0);
        Program_pause(0.1);
    }

    for (int i = 1; i <= 4; i++) {
        Finch_setTail(i, 25*i, 25*(4-i), 25);
        Program_pause(0.2);
    }
    Finch_setTailAll(0, 128, 255);
    Program_pause(0.2);
    Finch_setTailAll(0, 0, 0);
    Finch_setBeak(0, 0, 0);

    Finch_println("distance");
    Finch_clearDisplay();
    while (!Finch_getButton('T')) {
        Program_pause(0.2);
        int d = Finch_getDistance();
        for (int i = 0; i < 25; i++)
          Finch_setPoint(i/5, i%5, d >= i*20 ? 255 : 0);
    }

    Finch_println("light");
    Finch_clearDisplay();
    while (!Finch_getButton('A')) {
        Program_pause(0.1);
        // Finch_println(" > ");
        int a = Finch_getLight('R');
        // Finch_println(a);
        int b = Finch_getLight('L');
        for (int i = 0; i < 10; i++) {
          Finch_setPoint(i/2, i%2, a >= i ? 255 : 0);
          Finch_setPoint(i/2, 4-(i%2), b >= i ? 255 : 0);
        }
    }

    Finch_println("#");

    Finch_move('F', 30, 100); // forward, 30 cm, 100 percent speed
    Program_pause(0.25);
    Finch_move('B', 30, 20); // backward, 30 cm, 20 percent speed
    Finch_turn('L', 45, 50);
    Finch_motors(0, 15);
    Program_pause(2);
    Finch_motors(15, 0);
    Program_pause(2);
    Finch_stop();

    Finch_playNote(64, 0.25);
    Program_pause(0.25);
    Finch_playNote(66, 0.25);
    Program_pause(0.25);
    Finch_playNote(68, 0.25);

}

