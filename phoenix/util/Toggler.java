package org.firstinspires.ftc.teamcode.phoenix.util;


public class Toggler {

    private boolean lastState = false;
    private int integrated = 0;
    private int sensitive = 0;

    public Toggler() {
        // default constructor
    }

    public Toggler(int sensitive) {
        this.sensitive = sensitive;
    }

    public boolean shouldToggle(boolean currentBtnState) {
        if (currentBtnState && !lastState) {
            integrated++;
            if (integrated > sensitive) {
                integrated = (int) sensitive;
                lastState = true;
                return true;
            }
        } else
        if (!currentBtnState) {
            lastState = false;
            integrated = 0;
        }
        return false;
    }
}
