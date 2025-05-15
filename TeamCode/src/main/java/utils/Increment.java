package utils;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Increment {
    private final double slow;
    private final double medium;
    private final double fast;
    private final ElapsedTime elapsedTime;

    public Increment (double slow, double medium, double fast) {
        this.slow = slow;
        this.medium = medium;
        this.fast = fast;
        elapsedTime = new ElapsedTime();
    }

    public void reset() {
        elapsedTime.reset();
    }

    /**
     * Based on the elapsed time return a value to increment by
     * @return value to increment by
     */
    public double get() {

        int sleepTime;
        double delta;

        if (elapsedTime.seconds() < 3) {
            delta = slow;
            sleepTime = 500;
        } else if (elapsedTime.seconds() < 6) {
            delta = medium;
            sleepTime = 500;
        } else if (elapsedTime.seconds() < 9) {
            delta = fast;
            sleepTime = 500;
        } else {
            delta = fast;
            sleepTime = 250;
        }

        try {
            Thread.sleep(sleepTime);
        } catch (InterruptedException e) {
            return 0;
        }

        return delta;
    }
}
