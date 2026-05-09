package org.firstinspires.ftc.teamcode.utils;

/**
 * Generic pair container for two related values.
 * @param <F> type of the first value
 * @param <S> type of the second value
 */
public class Pair<F, S> {
    public final F first;
    public final S second;

    public Pair(F first, S second) {
        this.first  = first;
        this.second = second;
    }
}