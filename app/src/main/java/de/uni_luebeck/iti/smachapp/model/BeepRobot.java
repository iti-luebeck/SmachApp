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

    List<IntSensor> sensorsIR = new ArrayList<IntSensor>();

    List<ColorSensor> sensorsCol = new ArrayList<ColorSensor>();

    List<IntActuator> intActuators = new ArrayList<IntActuator>();
    List<ColorActuator> colorActuators = new ArrayList<ColorActuator>();

    Connection conn;
    Session sess;

    PrintWriter piCommand;

    public BeepRobot() {
        // Define default Beep sensors
        sensorsIR.add(new IntSensor("IR0", "topic/IR0", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR1", "topic/IR1", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR2", "topic/IR2", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR3", "topic/IR3", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR4", "topic/IR4", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR5", "topic/IR5", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR6", "topic/IR6", "Int8",
                "std_msgs.msg", "data", -128, 127));
        sensorsIR.add(new IntSensor("IR7", "topic/IR7", "Int8",
                "std_msgs.msg", "data", -128, 127));


        sensorsCol.add(new ColorSensor("UIR0", "topic/UIR0", "Int8",
                "std_msgs.msg", "data"));
        sensorsCol.add(new ColorSensor("UIR1", "topic/UIR2", "Int8",
                "std_msgs.msg", "data"));
        sensorsCol.add(new ColorSensor("UIR2", "topic/UIR3", "Int8",
                "std_msgs.msg", "data"));


        // Define default Beep actuators
        // actuators.add(new BeepActuator("MOTOR1", "topic/motors", "Motors",
        // "beep.msg", "links"));
        // actuators.add(new BeepActuator("MOTOR2", "topic/motors", "Motors",
        // "beep.msg", "rechts"));
        intActuators.add(new IntActuator("MOTOR_L", "motor_l", "Int8", "std_msgs.msg", "data", -128, 127));
        intActuators.add(new IntActuator("MOTOR_R", "motor_r", "Int8", "std_msgs.msg", "data", -128, 127));

        colorActuators.add(new ColorActuator("LED0", "topic/LED0", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED1", "topic/LED1", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED2", "topic/LED2", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED3", "topic/LED3", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED4", "topic/LED4", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED5", "topic/LED5", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED6", "topic/LED6", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED7", "topic/LED7", "Int8",
                "std_msgs.msg", "data"));
        colorActuators.add(new ColorActuator("LED8", "topic/LED8", "Int8",
                "std_msgs.msg", "data"));
        /*
        Actuator beep = new Actuator("BEEP", "topic/beep", "Int8",
				"std_msgs.msg", "data");
		actuators.add(beep);

	    */
    }

    public List<IntActuator> getIntActuators() {
        return intActuators;
    }

    public List<ColorActuator> getColorActuators() {
        return colorActuators;
    }

    public List<IntSensor> getIntSensors() {
        return sensorsIR;
    }

    public List<ColorSensor> getColorSensors() {
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
        act.addAll(intActuators);
        act.addAll(colorActuators);
        return act;
    }

    public boolean connect(String connectTo) {
        conn = new Connection(connectTo);//beep: "141.83.158.207"
        try {
            //connect and authorize
            conn.connect();
            conn.authenticateWithPassword("pi", "beep");

            //start a compatible shell
            sess = conn.openSession();
            sess.requestDumbPTY();
            sess.startShell();
            piCommand = new PrintWriter(sess.getStdin(), true);
            //echo to recognize when shell is ready to use
            piCommand.println("echo 'ready to start'");

            //print output and block until shell is "ready to start"
            InputStream stdout = new StreamGobbler(sess.getStdout());
            BufferedReader br = new BufferedReader(
                    new InputStreamReader(stdout));
            while (true) {
                String line = br.readLine();
                if (line == null) {
                    break;
                }
                System.out.println(line);
                if (line.equals("ready to start")) {
                    break;
                }
            }

            br.close();
            stdout.close();

            return true;
        } catch (Exception e) {
            e.printStackTrace();
        }
        return false;
    }

    public void disconnect() {
        piCommand.close();
        piCommand = null;
        sess.close();
        sess = null;
        conn.close();
        conn = null;
    }


    public void transmit(File f) {
        SCPClient client = new SCPClient(conn);
        try {
            client.put(f.getAbsolutePath(), "~/Beep/Software/catkin_ws/src/beep_imu");
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
    }

    public void play() {
        if (conn != null && sess != null && piCommand != null) {
            piCommand.println("mkdir -p ~/log");
            piCommand
                    .println("nohup roscore 2> ~/log/roscore-err.log 1> ~/log/roscore-out.log &");
            piCommand
                    .println("nohup rosrun beep_imu ir_distance.py 2> ~/log/ir_distance-err.log 1> ~/log/ir_distance-out.log");
        }
    }

    public String getRobotName() {
        return "Beep";
    }
}
