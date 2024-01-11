package org.firstinspires.ftc.teamcode.common.util.logging;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

public class Logger {

    public static class LogEntry {
        private String data;
        private double timestamp;

        public LogEntry(String data, double timestamp) {
            this.data = data;
            this.timestamp = timestamp;
        }

        public String getData() {
            return this.data;
        }

        public double getTimestamp() {
            return this.timestamp;
        }

        @Override
        public String toString() {
            return timestamp + ": " + data;
        }
    }

    private static List<BlockingQueue<LogEntry>> buffer;
    private final long timeSinceStart;
    private static Logger instance = new Logger();

    private Logger() {
        buffer = new ArrayList<>();
        for (int i = 0; i < LogType.values().length; i++) {
            buffer.add(new LinkedBlockingQueue<>());
        }
        timeSinceStart = System.currentTimeMillis();
    }

    public static Logger getInstance() {
        return instance;
    }

    public static void logData(LogType logType, String data) {
        double timestamp = (System.currentTimeMillis() - instance.timeSinceStart) / 1000.0;
        buffer.get(logType.ordinal()).add(new LogEntry(data, timestamp));
    }

    public static List<BlockingQueue<LogEntry>> getData() {
        return buffer;
    }

    public static void printData() {
        for (int i = 0; i < buffer.size(); i++) {
            System.out.println("NEW ENTRY - " + LogType.values()[i]);
            for (LogEntry entry : buffer.get(i)) System.out.println(entry.toString());
        }
    }
}