package org.firstinspires.ftc.teamcode.common.hardware;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.Map;

public class Logger {

    enum LogType {
        ODO_X, ODO_Y, ODO_H, ODO_EX, ODO_EY, ODO_EH, ARM_POS, ARM_ERR, EXT_POS, EXT_ERR
    }

    private long startTime;
    private Map<Long, LinkedList<LogEntry>> buffer;

    public Logger() {
        this.startTime = System.currentTimeMillis();
        this.buffer = new HashMap<>();
    }

    public void logData(LogType type, String data) {
        long currentTime = System.currentTimeMillis() - startTime;
        buffer.putIfAbsent(currentTime, new LinkedList<>());
        buffer.get(currentTime).add(new LogEntry(type, data));
    }

    private static class LogEntry {
        private LogType type;
        private String data;

        public LogEntry(LogType type, String data) {
            this.type = type;
            this.data = data;
        }

        @Override
        public String toString() {
            return "ENTRY - " + type + ": " + data;
        }
    }
}
