package de.uni_luebeck.iti.smachGenerator;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.NoSuchElementException;

/**
 * Creates the python Source-Code for a Smach state machine for ROS (Robot
 * Operation System) It can be stored to a file
 *
 * @author Marvin Lindenberg
 */
public class SmachAutomat {

    private ArrayList<? extends ISmachableState> states = new ArrayList<ISmachableState>();
    private LinkedList<String> smachStates = new LinkedList<String>();
    private int initialStateIndex;
    private SmachableSensors sensors;
    private SmachableActuators actuators;

    /**
     * Constructs a Smach state machine form the given states
     *
     * @param states used to create the state machine
     * @throws NoSuchElementException if sensors are used that are not specified in the
     *                                {@link SmachableSensors} object.
     */
    public SmachAutomat(ArrayList<? extends ISmachableState> states,
                        SmachableSensors sensors, SmachableActuators actuators)
            throws NoSuchElementException {
        this.states = states;
        this.sensors = sensors;
        this.actuators = actuators;
        for (ISmachableState s : states) {
            if (s.isInitialState()) {
                initialStateIndex = states.indexOf(s);
            }
            smachStates.add(getSmachState(s));
        }
    }

    private String getImports() {
        String imports = "#!/usr/bin/env python\n\n";
        imports += "import roslib; roslib.load_manifest('xxxxxx')\n";// TODO
        // Manifest
        // festlegen
        imports += "import rospy\n";
        imports += "import smach\n";
        imports += "import smach_ros\n\n";

        // messege dependencies
        HashSet<String> deps = sensors.getMsgDeps();
        deps.addAll(actuators.getMsgDeps());
        for (String dep : deps) {
            imports += dep + "\n";
        }
        return imports;
    }

    /**
     * Rewrites the ISmachableState s to a Smach state
     *
     * @param s ISmachableState that will be converted to a smach state
     * @throws NoSuchElementException
     */
    private String getSmachState(ISmachableState s)
            throws NoSuchElementException {
        String state = "class " + s.getName().replace(" ", "")
                + "(smach.State):\n";
        // create __init__ function for state
        state += "\tdef __init__(self):\n";
        state += "\t\tsmach.State.__init__(self, outcomes=[";
        // get transitions for __init__ function
        // a state
        if (s.getTransitions().size() > 0) {
            for (ISmachableTransition t : s.getTransitions()) {
                state += "'" + t.getLabel() + "',";
            }
            state = state.substring(0, state.length() - 1) + "])\n\n";
        } else {
            state += "])\n\n";
        }

        // create execute function for the state
        state += "\tdef execute(self, userdata):\n";
        state += "\t\trospy.loginfo('Executing state " + s.getName() + "')\n";
        for (ISmachableTransition t : s.getTransitions()) {
            ISmachableGuard g = t.getSmachableGuard();
            for (String sensor : g.getSensorNames()) {
                if (sensor.startsWith("DIFFERENCE_")) {
                    String sensorNames[] = sensor.replace("DIFFERENCE_", "")
                            .split("_");
                    if (sensors.getSensor(sensorNames[0]) != null
                            && sensors.getSensor(sensorNames[1]) != null) {
                        state += "\t\tglobal " + sensorNames[0]
                                + "\n\t\tglobal " + sensorNames[1] + "\n";
                    }
                }
                state += "\t\tglobal " + sensor + "\n";
            }
        }
        for (ISmachableActuator actuator : actuators) {
            state += "\t\tglobal pub_" + actuator.getTopic().replace("/", "_")
                    + "\n";
        }
        LinkedList<String> msgs = new LinkedList<String>();
        HashSet<String> publish = new HashSet<String>();
        for (ISmachableAction a : s.getActions()) {
            ISmachableActuator actuator = actuators.getActuator(a.getKey());
            if (!msgs.contains(actuator.getTopic().replace("/", "_") + " = "
                    + actuator.getTopicType() + "()")) {
                msgs.add(actuator.getTopic().replace("/", "_") + " = "
                        + actuator.getTopicType() + "()");
                state += "\t\t" + actuator.getTopic().replace("/", "_") + " = "
                        + actuator.getTopicType() + "()\n";
            }
            state += "\t\t" + actuator.getTopic().replace("/", "_") + "."
                    + actuator.getObjectInMessage() + " = " + a.getValue()
                    + "\n";
            publish.add("\t\tpub_" + actuator.getTopic().replace("/", "_")
                    + ".publish(" + actuator.getTopic().replace("/", "_")
                    + ")\n");
        }
        for (String pub : publish) {
            state += pub;
        }

        // check for transition
        state += "\n\t\twhile not rospy.is_shutdown():\n";
        for (ISmachableTransition t : s.getTransitions()) {
            ISmachableGuard guard = t.getSmachableGuard();
            if (guard.getSensorNames().size() > 0) {
                state += "\t\t\tif(";
                for (int i = 0; i < guard.getSensorNames().size(); i++) {
                    if (guard.getSensorNames().get(i).startsWith("DIFFERENCE_")) {
                        String sensorNames[] = guard.getSensorNames().get(i)
                                .replace("DIFFERENCE_", "").split("_");
                        if (sensors.getSensor(sensorNames[0]) != null
                                && sensors.getSensor(sensorNames[1]) != null) {
                            state += sensorNames[0] + "-" + sensorNames[1]
                                    + guard.getOperators().get(i)
                                    + guard.getCompValues().get(i) + " and ";
                        }
                    } else if (sensors.getSensor(guard.getSensorNames().get(i)) != null) {
                        state += sensors.getSensor(
                                guard.getSensorNames().get(i))
                                .getTransitionCondition(
                                        guard.getOperators().get(i),
                                        guard.getCompValues().get(i))
                                + " and ";
                    }
                }
                state = state.substring(0, state.length() - 5) + "):\n\t";
            }
            state += "\t\t\treturn '" + t.getLabel() + "'\n";
        }
        state += "\t\t\trospy.sleep(0.01)\n";
        return state;
    }

    private String getSmachStateMachine(String sm) {
        // prepare all states to add them to the Smach state machine
        ArrayList<String> stateToAdd = new ArrayList<String>();
        for (int i = 0; i < states.size(); i++) {
            String state = "smach.StateMachine.add('"
                    + states.get(i).getName().replace(" ", "") + "', "
                    + states.get(i).getName().replace(" ", "")
                    + "(), transitions={";
            for (ISmachableTransition t : states.get(i).getTransitions()) {
                state += "'" + t.getLabel() + "':'"
                        + t.getFollowerState().getName().replace(" ", "")
                        + "',";
            }
            if (states.get(i).getTransitions().size() > 0) {
                state = state.substring(0, state.length() - 1) + "})";
            } else {
                state += "})";
            }
            stateToAdd.add(state);
        }
        // create state machine and add states in correct order
        String stateMachine = "\t" + sm
                + " = smach.StateMachine(outcomes=[])\n";
        stateMachine += "\twith " + sm + ":\n";
        stateMachine += "\t\t" + stateToAdd.get(initialStateIndex) + "\n";
        for (int i = 0; i < stateToAdd.size(); i++) {
            if (i != initialStateIndex) {
                stateMachine += "\t\t" + stateToAdd.get(i) + "\n";
            }
        }
        return stateMachine;
    }

    private String getMainMethode() {
        String main = "if __name__ == '__main__':\n";
        main += "\trospy.init_node('zusmoro_state_machine')\n";
        for (String subSetup : sensors.getSubscriberSetups()) {
            main += "\t" + subSetup;
        }
        main += getSmachStateMachine("sm");
        // including possibility to use Smach_viewer
        main += "\tsis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')\n";
        main += "\tsis.start()\n\tsm.execute()\n";
        main += "\trospy.spin()\n\tsis.stop()";
        return main;
    }

    /**
     * Stores the smach automat as an executable python-program to the given
     * filename
     *
     * @param file the python-programm will be stored in.
     * @return true, if constructing and saving was successfully. False if there
     * were problems with Topics, names, or the saving progress.
     * @throws NoSuchElementException
     * @throws java.lang.IllegalArgumentException
     */
    public boolean saveToFile(File file) {
        try {
            String pythonNode = getImports() + "\n\n";
            // define global sensor variables
            for (ISmachableSensor sensor : sensors) {
                pythonNode += sensor.getKey() + " = 0\n";
            }
            // define global actuator publisher
            for (String pub : actuators.getPublisherSetups()) {
                pythonNode += pub + "\n";
            }

            pythonNode += "\n\n";

            for (int i = 0; i < smachStates.size(); i++) {
                pythonNode += smachStates.get(i) + "\n";
            }
            for (String cb : sensors.getCallbacks()) {
                pythonNode += cb + "\n";
            }
            pythonNode += getMainMethode();
            PrintWriter out = new PrintWriter(file);
            out.print(pythonNode);
            out.close();
            file.setExecutable(true);
        } catch (Exception e) {
            e.printStackTrace();
            return false;
        }
        return true;
    }

}
