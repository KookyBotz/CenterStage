package org.firstinspires.ftc.teamcode.common.util.logging;

import android.os.Environment;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.StringJoiner;
import java.util.concurrent.BlockingQueue;

public class CSVInterface {
    private static String LOG_DIRECTORY;

    public static void log() {
        LOG_DIRECTORY = Environment.getExternalStorageDirectory().getAbsolutePath() + "/FIRST/" + new SimpleDateFormat("yyyy_MM_dd_HH_mm_ss").format(new Date()) + ".csv";
        StringBuilder fileContent = new StringBuilder();

        try {
            // Header
            StringJoiner headerJoiner = new StringJoiner(",");
            for (LogType type : LogType.values()) {
                headerJoiner.add("time (s)," + type.getHeader() + " (" + type.getUnit() + ")");
            }
            fileContent.append(headerJoiner.toString()).append("\n");

            // Data
            List<BlockingQueue<Logger.LogEntry>> data = Logger.getData();
            boolean moreData;
            do {
                moreData = false;
                StringJoiner lineJoiner = new StringJoiner(",");
                for (BlockingQueue<Logger.LogEntry> queue : data) {
                    Logger.LogEntry entry = queue.poll();
                    if (entry != null) {
                        lineJoiner.add(entry.getTimestamp() + "," + entry.getData());
                        moreData = true;
                    } else {
                        lineJoiner.add(" , ");
                    }
                }
                fileContent.append(lineJoiner.toString()).append("\n");
            } while (moreData);

            // Write to file
            try (BufferedWriter writer = new BufferedWriter(new FileWriter(LOG_DIRECTORY))) {
                writer.write(fileContent.toString());
            }
        } catch (IOException e) {

            e.printStackTrace();
            throw new ArithmeticException("math aint mathing");
        }
    }
}