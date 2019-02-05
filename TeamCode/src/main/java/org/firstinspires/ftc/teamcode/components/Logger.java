package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.components.utilities.TrcDbgTrace;

public class Logger {

    private boolean traceEnabled = false;
    private boolean fileOpen = false;
    private String filePrefix = "Logger";
    private TrcDbgTrace tracer;

    /* Constructor */
    public Logger(String aFilePrefix) {
        filePrefix = aFilePrefix;
        open(traceEnabled);
    }

    public void open(boolean enableTrace) {
        open(filePrefix, enableTrace);
    }

    public void open(String filePrefix, boolean enableTrace) {
        traceEnabled = enableTrace;
        if (traceEnabled && !fileOpen) {
            tracer = new TrcDbgTrace("LOGGER", traceEnabled, TrcDbgTrace.TraceLevel.HIFREQ, TrcDbgTrace.MsgLevel.VERBOSE);
            tracer.openTraceLog("/sdcard/FIRST/tracelog", filePrefix);
            fileOpen = true;
        }
    }

    public void logErr(final String funcName, final String format, Object... args) {
        if (traceEnabled) {
            tracer.traceErr(funcName, format, args);
        }
    }

    public void logInfo(final String funcName, final String format, Object... args) {
        if (traceEnabled) {
            tracer.traceInfo(funcName, format, args);
        }
    }

    public void logDebug(final String funcName, final String format, Object... args) {
        if (traceEnabled) {
            tracer.traceVerbose(funcName, format, args);
        }
    }

    public void close() {
        if (traceEnabled) {
            tracer.closeTraceLog();
        }
    }

}
