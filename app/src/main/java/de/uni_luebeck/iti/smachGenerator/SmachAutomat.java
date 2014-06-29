package de.uni_luebeck.iti.smachGenerator;

import java.io.File;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.NoSuchElementException;

/**
 * Creates the python Source-Code for a Smach state machine for ROS (Robot
 * Operation System) It can be stored to a file.
 */
public class SmachAutomat {

    private List<? extends ISmachableState> states;
    private LinkedList<String> smachStates = new LinkedList<String>();
    private int initialStateIndex = 0;
    private SmachableSensors sensors;
    private SmachableActuators actuators;
    private String pkg;

    /**
     * Constructs a Smach state machine form the given states
     *
     * @param states    used to create the state machine
     * @param sensors   containing all {@link ISmachableSensor}s that are used in the
     *                  {@link ISmachableTransition}s
     * @param actuators containing all {@link ISmachableActuator}s that are used in
     *                  the {@link ISmachableState}s
     * @param pkg       the package the state machine is stored in, to load the right
     *                  manifest
     * @throws NoSuchElementException if sensors are used that are not specified in the
     *                                {@link SmachableSensors} object.
     */
    public SmachAutomat(List<? extends ISmachableState> states,
                        SmachableSensors sensors, SmachableActuators actuators, String pkg)
            throws NoSuchElementException {
        this.states = states;
        this.sensors = sensors;
        this.actuators = actuators;
        this.pkg = pkg;
        for (ISmachableState s : states) {
            if (s.isInitialState()) {
                initialStateIndex = states.indexOf(s);
            }
            smachStates.add(getSmachState(s));
        }
    }

    private String getImports() {
        String imports = "#!/usr/bin/env python\n\n";
        imports += "import roslib; roslib.load_manifest('" + pkg + "')\n";
        imports += "import rospy\n";
        imports += "import smach\n";
        imports += "import smach_ros\n";
        imports += "import atexit\n\n";

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

        // add global declaration for all sensor variables
        for (String glob : sensors.getGlobalIdentifiers()) {
            state += "\t\tglobal " + glob + "\n";
        }
        // add global declaration for all publisher
        for (String publ : actuators.getGlobalIdentifiers()) {
            state += "\t\tglobal " + publ + "\n";
        }

        // add actions
        for (ISmachableAction a : s.getActions()) {
            ISmachableActuator actuator = actuators.getActuator(a.getActuatorName());
            for (String str : actuator.getPublishMessage(a)) {
                state += "\t\t" + str + "\n";
            }
        }

        // add transition conditions
        state += "\n\t\twhile not rospy.is_shutdown():\n";
        for (ISmachableTransition t : s.getTransitions()) {
            ISmachableGuard guard = t.getSmachableGuard();
            if (guard.getSensorNames().size() > 0) {
                state += "\t\t\tif(";
                for (int i = 0; i < guard.getSensorNames().size(); i++) {
                    // add condition for each guard of this transition
                    if (guard.getSensorNames().get(i).startsWith("DIFFERENCE_")) {
                        String sensorNames[] = guard.getSensorNames().get(i)
                                .replace("DIFFERENCE_", "").split("_");
                        ISmachableSensor sensor1 = sensors
                                .getSensor(sensorNames[0]);
                        ISmachableSensor sensor2 = sensors
                                .getSensor(sensorNames[1]);
                        state += sensor1.getValueIdentifier() + "-"
                                + sensor2.getValueIdentifier()
                                + guard.getOperators().get(i)
                                + guard.getCompValues().get(i) + " and ";
                    } else {
                        ISmachableSensor sensor = sensors.getSensor(guard
                                .getSensorNames().get(i));
                        if (sensor != null) {
                            state += sensor.getTransitionCondition(guard
                                    .getOperators().get(i) + "", guard
                                    .getCompValues().get(i))
                                    + " and ";
                        } else {
                            throw new NoSuchElementException(
                                    "Zugriff auf unbekannten Sensor: "
                                            + guard.getSensorNames().get(i)
                            );
                        }
                    }
                }
                state = state.substring(0, state.length() - 5) + "):\n\t";
            }
            state += "\t\t\treturn '" + t.getLabel() + "'\n";
        }
        state += "\t\t\trospy.sleep(0.01)\n";
        return state;
    }

    /**
     * create the smachMachine
     *
     * @param sm the identifier of the state machine
     * @return
     */
    private String getSmachStateMachine(String sm) {
        if (states.size() > 0) {
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
            String stateMachine = "\t\t" + sm
                    + " = smach.StateMachine(outcomes=[])\n";
            stateMachine += "\t\twith " + sm + ":\n";
            stateMachine += "\t\t\t" + stateToAdd.get(initialStateIndex) + "\n";
            for (int i = 0; i < stateToAdd.size(); i++) {
                if (i != initialStateIndex) {
                    stateMachine += "\t\t\t" + stateToAdd.get(i) + "\n";
                }
            }
            return stateMachine;
        }
        return "";
    }

    private String getMainMethod() {
        String main = "if __name__ == '__main__':\n";
        main += "\ttry:\n";
        main += "\t\trospy.init_node('zusmoro_state_machine', disable_signals=True)\n";
        for (String subSetup : sensors.getSubscriberSetups()) {
            main += "\t\t" + subSetup + "\n";

        }
        main += getSmachStateMachine("sm");
        // including possibility to use Smach_viewer
        main += "\t\tsis = smach_ros.IntrospectionServer('Beep_State_Server', sm, '/SM_ROOT')\n";
        main += "\t\tsis.start()\n\t\tsm.execute()\n";
        main += "\t\trospy.spin()\n\t\tsis.stop()\n";
        //add shutdown sequence
        main += "\tfinally:\n";
        main += "\t\trospy.loginfo('zusmoro_state_machine is shutting down')\n";
        for (ISmachableActuator act : actuators) {
            for (String com : act.onShutDown()) {
                main += "\t\t" + com + "\n";
            }
        }
        main += "\t\trospy.signal_shutdown('zusmoro_state_machine was terminated by KeyBoard Interupt')\n";
        return main;
    }

    /**
     * Stores the smach automat as an executable python-program to the given
     * filename
     *
     * @param file the python-programm will be stored in.
     * @return true, if constructing and saving was successfully. False if there
     * were problems with Topics, names, or the saving progress.
     * @throws NoSuchElementException if there are no states in the states handed over in the
     *                                constructor
     */
    public boolean saveToFile(File file) throws NoSuchElementException {
        if (states.size() == 0) {
            throw new NoSuchElementException(
                    "Tried to create an smach automat out of NO STATES!");
        }
        try {
            String pythonNode = getImports() + "\n\n";
            // define global sensor variables
            for (String init : sensors.getIdentifierInit()) {
                pythonNode += init + "\n";
            }
            // define global actuator publisher
            for (String pub : actuators.getPublisherSetups()) {
                pythonNode += pub + "\n";
            }
            pythonNode += "\n\n";

            // add states
            for (int i = 0; i < smachStates.size(); i++) {
                pythonNode += smachStates.get(i) + "\n";
            }
            // add callbacks
            for (String cb : sensors.getCallbacks()) {
                pythonNode += cb + "\n";
            }
            // add main method
            pythonNode += getMainMethod();
            // save to file
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
