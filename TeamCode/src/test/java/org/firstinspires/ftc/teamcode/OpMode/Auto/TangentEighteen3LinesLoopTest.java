//package org.firstinspires.ftc.teamcode.OpMode.Auto;
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.math.Vector;
//import com.pedropathing.util.Timer;
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.hardware.lynx.LynxModule;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.Config.Logging.MatchLogger;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.IntakeWithLogging;
//import org.firstinspires.ftc.teamcode.Config.Subsystems.NewShooterWithLogging;
//import org.junit.Before;
//import org.junit.Test;
//
//import java.lang.reflect.Field;
//import java.util.Collections;
//
//import static org.junit.Assert.assertEquals;
//import static org.mockito.Mockito.CALLS_REAL_METHODS;
//import static org.mockito.Mockito.mock;
//import static org.mockito.Mockito.when;
//
//public class TangentEighteen3LinesLoopTest {
//
//    private static final int LOOP_ITERATIONS = 10;
//
//    private TangentEighteen3Lines opMode;
//    private MatchLogger logger;
//
//    @Before
//    public void setUp() throws Exception {
//        opMode = mock(TangentEighteen3Lines.class, CALLS_REAL_METHODS);
//
//        Follower mockFollower              = mock(Follower.class);
//        NewShooterWithLogging mockShooter  = mock(NewShooterWithLogging.class);
//        IntakeWithLogging mockIntake        = mock(IntakeWithLogging.class);
//        GoBildaPinpointDriver mockPinpoint = mock(GoBildaPinpointDriver.class);
//        Timer mockPathTimer                = mock(Timer.class);
//        LynxModule mockHub                 = mock(LynxModule.class);
//
//        // Pose is final — use a rexal instance. Vector has a no-arg constructor.
//        when(mockFollower.getPose()).thenReturn(new Pose());
//        when(mockFollower.getVelocity()).thenReturn(new Vector());
//
//        logger = new MatchLogger();
//
//        opMode.telemetry = mock(Telemetry.class);
//
//        setField(opMode, "follower",       mockFollower);
//        setField(opMode, "shooter",        mockShooter);
//        setField(opMode, "intake",         mockIntake);
//        setField(opMode, "pinpoint",       mockPinpoint);
//        setField(opMode, "pathTimer",      mockPathTimer);
//        setField(opMode, "logger",         logger);
//        setField(opMode, "allHubs",        Collections.singletonList(mockHub));
//        // beamWasCleared has a field initializer of `true` that Objenesis skips.
//        setField(opMode, "beamWasCleared", true);
//    }
//
//    @Test
//    public void loop_runsMultipleIterations() {
//        opMode.start();
//
//        for (int i = 0; i < LOOP_ITERATIONS; i++) {
//            opMode.loop();
//        }
//
//        opMode.stop();
//
//        assertEquals(LOOP_ITERATIONS, logger.getRowCount());
//    }
//
//    private static void setField(Object target, String name, Object value) throws Exception {
//        Class<?> clazz = target.getClass();
//        while (clazz != null) {
//            try {
//                Field f = clazz.getDeclaredField(name);
//                f.setAccessible(true);
//                f.set(target, value);
//                return;
//            } catch (NoSuchFieldException e) {
//                clazz = clazz.getSuperclass();
//            }
//        }
//        throw new NoSuchFieldException("Field not found in class hierarchy: " + name);
//    }
//}
