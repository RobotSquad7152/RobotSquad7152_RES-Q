package RobotSquad;

/**
 * Created by Tony on 9/27/2015.
 */
public class Stopwatch {

    private long start;

    /**
     * Initializes a new stopwatch.
     */
    public Stopwatch() {
        start = System.currentTimeMillis();
    }

    public void Resetstopwatch() { start = System.currentTimeMillis();}
    /**
     * Returns the elapsed CPU time (in seconds) since the stopwatch was created.
     *
     * @return elapsed CPU time (in seconds) since the stopwatch was created
     */
    public long elapsedTime() {
        long now = System.currentTimeMillis();
        return (now - start);
    }

}

