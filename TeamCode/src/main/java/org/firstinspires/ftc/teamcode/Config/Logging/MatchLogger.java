package org.firstinspires.ftc.teamcode.Config.Logging;

import com.qualcomm.robotcore.util.RobotLog;

import java.util.ArrayList;
import java.util.List;

public class MatchLogger {

    private static final String TAG = "MatchLog";
    private static final int MAX_ROWS = 10000;

    private final List<LogRow> rows = new ArrayList<>(MAX_ROWS);
    private LogRow currentRow;
    private long startNs;
    private int loopCount;

    public void start() {
        startNs = System.nanoTime();
    }

    public void beginRow() {
        currentRow = new LogRow();
        currentRow.loopCount = loopCount++;
        currentRow.timestampNs = System.nanoTime() - startNs;
    }

    public LogRow getCurrentRow() {
        return currentRow;
    }

    public void commitRow() {
        if (currentRow != null && rows.size() < MAX_ROWS) {
            rows.add(currentRow);
        }
        currentRow = null;
    }

    public void dump() {
        RobotLog.ii(TAG, LogRow.csvHeader());
        for (LogRow row : rows) {
            RobotLog.ii(TAG, row.toCsvLine());
        }
    }

    public int getRowCount() {
        return rows.size();
    }

    public LogRow getRow(int index) {
        return rows.get(index);
    }
}
