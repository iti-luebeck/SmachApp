package de.uni_luebeck.iti.smachapp.model;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;

import ch.ethz.ssh2.Connection;
import ch.ethz.ssh2.SCPClient;
import ch.ethz.ssh2.Session;
import ch.ethz.ssh2.StreamGobbler;
import de.uni_luebeck.iti.smachGenerator.SmachableActuators;
import de.uni_luebeck.iti.smachGenerator.SmachableSensors;


public class BeepRobot {

    public static final String NAME = "BEEP";

    private List<BeepIRSensor> sensorsIR = new ArrayList<BeepIRSensor>();

    private List<BeepColorSensor> sensorsCol = new ArrayList<BeepColorSensor>();

    private List<BeepMotorActuator> motorActuators = new ArrayList<BeepMotorActuator>();
    private List<BeepColorRGBActuator> colorRGBActuators = new ArrayList<BeepColorRGBActuator>();

    private Connection conn;
    private Session sess;

    private String piDirAutomat;
    private String automatFileName;
    private PrintWriter piIn;
    private BufferedReader piOut;
    private boolean connected;


    public BeepRobot() {
        sensorsIR.add(new BeepIRSensor("IR0", "/IR_filtered", 0));
        sensorsIR.add(new BeepIRSensor("IR1", "/IR_filtered", 1));
        sensorsIR.add(new BeepIRSensor("IR2", "/IR_filtered", 2));
        sensorsIR.add(new BeepIRSensor("IR3", "/IR_filtered", 3));
        sensorsIR.add(new BeepIRSensor("IR4", "/IR_filtered", 4));
        sensorsIR.add(new BeepIRSensor("IR5", "/IR_filtered", 5));
        sensorsIR.add(new BeepIRSensor("IR6", "/IR_filtered", 6));
        sensorsIR.add(new BeepIRSensor("IR7", "/IR_filtered", 7));
        sensorsCol.add(new BeepColorSensor("UIR0", "/ground_colors", 0));
        sensorsCol.add(new BeepColorSensor("UIR1", "/ground_colors", 1));
        sensorsCol.add(new BeepColorSensor("UIR2", "/ground_colors", 2));
        //beepTimer = new BeepSensorTimer("timer");
        // smachableSensors.add(beepTimer);

        // Define default Beep actuators
        motorActuators.add(new BeepMotorActuator("MOTOR_L", "/motor_l"));
        motorActuators.add(new BeepMotorActuator("MOTOR_R", "/motor_r"));

        colorRGBActuators.add(new BeepColorRGBActuator("LED0", "/leds", 0));
        colorRGBActuators.add(new BeepColorRGBActuator("LED1", "/leds", 1));
        colorRGBActuators.add(new BeepColorRGBActuator("LED2", "/leds", 2));
        colorRGBActuators.add(new BeepColorRGBActuator("LED3", "/leds", 3));
        colorRGBActuators.add(new BeepColorRGBActuator("LED4", "/leds", 4));
        colorRGBActuators.add(new BeepColorRGBActuator("LED5", "/leds", 5));
        colorRGBActuators.add(new BeepColorRGBActuator("LED6", "/leds", 6));
        colorRGBActuators.add(new BeepColorRGBActuator("LED7", "/leds", 7));

        //motors.add(new BeepMotorActuator("BEEP", "/beep"));

        piDirAutomat = "/home/pi/ros/beep_framework/zusmoro_state_machine";
    }

    public String getRobotName() {
        return NAME;
    }

    public List<BeepMotorActuator> getMotorActuators() {
        return motorActuators;
    }

    public List<BeepColorRGBActuator> getColorRGBActuators() {
        return colorRGBActuators;
    }

    public List<BeepIRSensor> getIntSensors() {
        return sensorsIR;
    }

    public List<BeepColorSensor> getColorSensors() {
        return sensorsCol;
    }

    public SmachableSensors getSensors() {
        SmachableSensors sen = new SmachableSensors();
        sen.addAll(sensorsIR);
        sen.addAll(sensorsCol);
        return sen;
    }

    public SmachableActuators getActuators() {
        SmachableActuators act = new SmachableActuators();
        act.addAll(motorActuators);
        act.addAll(colorRGBActuators);
        return act;
    }

    public boolean connect(String connectTo) {

        conn = new Connection(connectTo);
        try {
            // connect and authorize
            conn.connect();

            conn.authenticateWithPassword("pi", "beep");

            // start a compatible shell
            sess = conn.openSession();
            sess.requestDumbPTY();
            sess.startShell();

            piIn = new PrintWriter(sess.getStdin(), true);

            // echo to recognize when shell is ready to use
            piIn.println("echo 'ready to start'");

            // print output and block until shell is "ready to start"
            InputStream stdout = new StreamGobbler(sess.getStdout());
            piOut = new BufferedReader(new InputStreamReader(stdout));
            String line = piOut.readLine();
            while (line != null) {
                System.out.println(line);
                if (line.equals("ready to start")) {
                    break;
                }
                line = piOut.readLine();
            }

            connected = true;

            // Start a new roscore
            piIn.println("mkdir -p ~/log");
            try {
                piOut.readLine(); // command
            } catch (IOException e) {
                e.printStackTrace();
            }
            if (!isRoscoreRunning()) {
                startNewRoscore();
                try {
                    Thread.sleep(4000);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }
            }
            startBeepNode();

            try {
                Thread.sleep(8000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            return true;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return false;
    }

    public void disconnect() {
        connected = false;
        piIn.close();
        piIn = null;
        try {
            piOut.close();
            piOut = null;
            sess.close();
            sess = null;
            conn.close();
            conn = null;
        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void stop() {
        if (connected) {
            try {
                piIn.println("pkill -2 -f 'python " + piDirAutomat + "/"
                        + automatFileName + "'");
                piOut.readLine(); // command
            } catch (IOException e) {
                e.printStackTrace();
            }

        }

    }

    public boolean transmit(File file) {

        automatFileName = file.getName();

        try {
            SCPClient client = new SCPClient(conn);
            client.put(file.getAbsolutePath(), piDirAutomat);
            piIn.println("chmod +x " + piDirAutomat + "/" + automatFileName);
            return true;
        } catch (IOException e) {
            e.printStackTrace();
            return false;
        }
    }

    public void play() {
        if (connected) {
            if (isRoscoreRunning()) {
                startAutomatOnPi();
            } else {
                startNewRoscore();
                startBeepNode();
                startAutomatOnPi();
            }
        }
    }

    /**
     * checks, if there is a running roscore process on the Beep
     *
     * @return true, if roscore is running
     */
    private boolean isRoscoreRunning() {
        System.out.print("Roscore running: ");
        if (connected) {
            piIn.println("ps -ef | grep 'roscore' | grep -v 'grep' | wc -l");
            try {
                String line = piOut.readLine();
                while (line.length() != 1) {
                    line = piOut.readLine();
                }
                if (line.equals("1")) { // number of roscore
                    // processes running
                    System.out.println("true");
                    return true;
                }
            } catch (Exception e) {
                e.printStackTrace();
            }

        }

        System.out.println("false");
        return false;
    }

    /**
     * Terminates every running roscore process and starts a new roscore. Mind,
     * that the roscore needs some time to start up after the call of this
     * function.
     */
    private void startNewRoscore() {
        if (connected) {
            piIn.println("pkill roscore");
            System.out.println("Starting new Roscore");
            piIn.println("nohup roscore 2> ~/log/roscore-err.log 1> ~/log/roscore-out.log &");
        }
    }

    /**
     * Starts a new _Beep node. Mind, that the node needs some time to start up after the call of this
     * function.
     */
    private void startBeepNode() {
        if (connected) {
            System.out.println("Starting Beep-node");
            piIn.println("nohup rosrun Beep_main_node beep.py  2> ~/log/beepNode-err.log 1> ~/log/beepNode-out.log &");
        }
    }

    /**
     * Stops all already running Smach automates by calling stop(). Then
     * starts the Smach automate <code>automateFileName</code> on Beep. Will
     * store output- and error stream to log files in <code>~/log</code> on Beep
     * <p/>
     * Be sure that there is a roscore running before calling this method.
     */
    private void startAutomatOnPi() {
        stop(); // Stop old automate
        piIn.println("nohup rosrun zusmoro_state_machine " + automatFileName
                + " 2> ~/log/" + automatFileName + "-err.log 1> ~/log/"
                + automatFileName + "-out.log &");
        try {
            piOut.readLine(); // command
            piOut.readLine(); // Process ID
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    public boolean isConnected(){
        return connected;
    }
}
