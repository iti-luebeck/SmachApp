package de.uni_luebeck.iti.smachapp.model;

import android.graphics.PointF;
import android.os.Environment;
import android.util.Xml;

import org.xmlpull.v1.XmlPullParser;
import org.xmlpull.v1.XmlPullParserException;
import org.xmlpull.v1.XmlSerializer;

import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.FilenameFilter;
import java.io.IOException;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by Morten Mey on 28.06.2014.
 */
public class XMLSaverLoader {

    public static final File PATH = new File(Environment.getExternalStorageDirectory(), "SMACH");
    public static final String FILE_ENDING = ".smach";
    public static final String PYTHON_FILE_ENDING = ".py";

    private static final String TAG_STATEMACHINE = "StateMachine";
    private static final String TAG_STATE = "State";
    private static final String TAG_ACTION = "Action";
    private static final String TAG_TRANSITION = "Transition";
    private static final String TAG_GUARD = "Guard";
    private static final String TAG_GUARD_ELEM = "GuardElement";
    private static final String TAG_PATH = "Path";
    private static final String TAG_POINT = "Point";

    private static final String PROP_NAME = "name";
    private static final String PROP_ROBOT = "robot";
    private static final String PROP_X = "x";
    private static final String PROP_Y = "y";
    private static final String PROP_IS_INITIAL_STATE = "isInitialState";
    private static final String PROP_ACTUATOR = "actuator";
    private static final String PROP_VALUE = "value";
    private static final String PROP_PREV = "previousState";
    private static final String PROP_NEXT = "nextState";
    private static final String PROP_SENSOR = "sensor";
    private static final String PROP_OPERATOR = "operator";

    public static File[] getAllSaves() {
        return PATH.listFiles(new FilenameFilter() {
            @Override
            public boolean accept(File file, String s) {
                return s.endsWith(FILE_ENDING);
            }
        });
    }

    public static boolean doesFileExist(String automatName) {
        File[] files = XMLSaverLoader.getAllSaves();

        String fileName = automatName + FILE_ENDING;

        boolean alreadyExists = false;
        for (File f : files) {
            if (f.getName().equals(fileName)) {
                alreadyExists = true;
                break;
            }
        }

        return alreadyExists;
    }

    public static void save(EditorModel model) throws IOException {
        PATH.mkdirs();
        XmlSerializer ser = Xml.newSerializer();
        ser.setOutput(new FileWriter(model.getSaveFile()));

        ser.startDocument(null, null);
        ser.startTag(null, TAG_STATEMACHINE);
        ser.attribute(null, PROP_NAME, model.getStateMachine().getName());
        ser.attribute(null, PROP_ROBOT, model.getRobot().getRobotName());

        for (State s : model.getStateMachine()) {
            saveState(s, ser);
        }

        for (Transition t : model.getStateMachine().getTransitions()) {
            saveTransition(t, ser);
        }

        ser.endTag(null, TAG_STATEMACHINE);
        ser.endDocument();
    }

    private static void saveState(State s, XmlSerializer ser) throws IOException {
        ser.startTag(null, TAG_STATE);
        ser.attribute(null, PROP_NAME, s.getName());
        ser.attribute(null, PROP_X, Float.toString(s.getX()));
        ser.attribute(null, PROP_Y, Float.toString(s.getY()));
        ser.attribute(null, PROP_IS_INITIAL_STATE, Boolean.toString(s.isInitialState()));

        for (Action a : s.getActions()) {
            saveAction(a, ser);
        }

        ser.endTag(null, TAG_STATE);
    }

    private static void saveAction(Action a, XmlSerializer ser) throws IOException {
        ser.startTag(null, TAG_ACTION);
        ser.attribute(null, PROP_ACTUATOR, a.getActuatorName());
        ser.attribute(null, PROP_VALUE, Integer.toString(a.getValue()));
        ser.endTag(null, TAG_ACTION);
    }

    private static void saveTransition(Transition t, XmlSerializer ser) throws IOException {
        ser.startTag(null, TAG_TRANSITION);
        ser.attribute(null, PROP_NAME, t.getLabel());
        ser.attribute(null, PROP_PREV, t.getPreviousState().getName());
        ser.attribute(null, PROP_NEXT, t.getFollowerState().getName());
        saveGuard(t.getSmachableGuard(), ser);
        savePath(t.getPath(), ser);
        ser.endTag(null, TAG_TRANSITION);
    }

    private static void saveGuard(Guard guard, XmlSerializer ser) throws IOException {
        ser.startTag(null, TAG_GUARD);
        for (int i = 0; i < guard.getCompValues().size(); i++) {
            ser.startTag(null, TAG_GUARD_ELEM);
            ser.attribute(null, PROP_SENSOR, guard.getSensorNames().get(i));
            ser.attribute(null, PROP_OPERATOR, guard.getOperators().get(i));
            ser.attribute(null, PROP_VALUE, Integer.toString(guard.getCompValues().get(i)));
            ser.endTag(null, TAG_GUARD_ELEM);
        }
        ser.endTag(null, TAG_GUARD);
    }

    private static void savePath(BezierPath path, XmlSerializer ser) throws IOException {
        ser.startTag(null, TAG_PATH);

        for (PointF point : path.getPoints()) {
            ser.startTag(null, TAG_POINT);
            ser.attribute(null, PROP_X, Float.toString(point.x));
            ser.attribute(null, PROP_Y, Float.toString(point.y));
            ser.endTag(null, TAG_POINT);
        }

        ser.endTag(null, TAG_PATH);
    }

    public static EditorModel load(File f) throws IOException, XmlPullParserException {
        XmlPullParser pars = Xml.newPullParser();
        pars.setFeature(XmlPullParser.FEATURE_PROCESS_NAMESPACES, false);
        pars.setInput(new FileReader(f));


        pars.nextTag();
        pars.require(XmlPullParser.START_TAG, null, TAG_STATEMACHINE);
        String name = pars.getAttributeValue(null, PROP_NAME);
        String robot = pars.getAttributeValue(null, PROP_ROBOT);

        EditorModel model = new EditorModel(name);

        if (!BeepRobot.NAME.equals(robot)) {
            throw new IllegalArgumentException("Unknown Robot");
        }

        while (pars.next() != XmlPullParser.END_TAG) {
            if (pars.getEventType() != XmlPullParser.START_TAG) {
                continue;
            }
            String tag = pars.getName();
            // Starts by looking for the entry tag
            if (tag.equals(TAG_STATE)) {
                loadState(pars, model);
            } else if (tag.equals(TAG_TRANSITION)) {
                loadTransition(pars, model);
            } else {
                skip(pars);
            }
        }

        return model;
    }

    private static void loadState(XmlPullParser pars, EditorModel model) throws XmlPullParserException, IOException {
        pars.require(XmlPullParser.START_TAG, null, TAG_STATE);
        String name = pars.getAttributeValue(null, PROP_NAME);
        float x = Float.parseFloat(pars.getAttributeValue(null, PROP_X));
        float y = Float.parseFloat(pars.getAttributeValue(null, PROP_Y));
        boolean isInit = Boolean.parseBoolean(pars.getAttributeValue(null, PROP_IS_INITIAL_STATE));

        State s = new State(name, x, y, isInit);
        model.getStateMachine().addState(s);

        while (pars.next() != XmlPullParser.END_TAG) {
            if (pars.getEventType() != XmlPullParser.START_TAG) {
                continue;
            }
            String tag = pars.getName();
            // Starts by looking for the entry tag
            if (tag.equals(TAG_ACTION)) {
                loadAction(pars, s);
            } else {
                skip(pars);
            }
        }

        pars.require(XmlPullParser.END_TAG, null, TAG_STATE);
    }

    private static void loadAction(XmlPullParser pars, State s) throws XmlPullParserException, IOException {
        pars.require(XmlPullParser.START_TAG, null, TAG_ACTION);
        String actuator = pars.getAttributeValue(null, PROP_ACTUATOR);
        int val = Integer.parseInt(pars.getAttributeValue(null, PROP_VALUE));
        s.addAction(new Action(actuator, val));
        pars.nextTag();
        pars.require(XmlPullParser.END_TAG, null, TAG_ACTION);
    }

    private static void loadTransition(XmlPullParser pars, EditorModel model) throws XmlPullParserException, IOException {
        pars.require(XmlPullParser.START_TAG, null, TAG_TRANSITION);
        String name = pars.getAttributeValue(null, PROP_NAME);
        String prev = pars.getAttributeValue(null, PROP_PREV);
        String next = pars.getAttributeValue(null, PROP_NEXT);

        State prevS = model.getStateMachine().getState(prev);
        State nextS = model.getStateMachine().getState(next);

        if (prevS == null || nextS == null) {
            throw new IllegalArgumentException("Illegal Transition one of the States does not exist.");
        }

        Guard g = null;
        BezierPath p = null;

        while (pars.next() != XmlPullParser.END_TAG) {
            if (pars.getEventType() != XmlPullParser.START_TAG) {
                continue;
            }
            String tag = pars.getName();
            // Starts by looking for the entry tag
            if (tag.equals(TAG_GUARD)) {
                g = loadGuard(pars);
            } else if (tag.equals(TAG_PATH)) {
                p = loadPath(pars);
            } else {
                skip(pars);
            }
        }

        if (g == null || p == null) {
            throw new IllegalArgumentException("Illegal Transition Guard or Parth was not set");
        }

        model.getStateMachine().addTransition(new Transition(prevS, nextS, name, p, g));

        pars.require(XmlPullParser.END_TAG, null, TAG_TRANSITION);
    }

    private static Guard loadGuard(XmlPullParser pars) throws XmlPullParserException, IOException {
        pars.require(XmlPullParser.START_TAG, null, TAG_GUARD);
        Guard g = new Guard();

        while (pars.next() != XmlPullParser.END_TAG) {
            if (pars.getEventType() != XmlPullParser.START_TAG) {
                continue;
            }
            String tag = pars.getName();
            // Starts by looking for the entry tag
            if (tag.equals(TAG_GUARD_ELEM)) {
                loadGuardElement(pars, g);
            } else {
                skip(pars);
            }
        }

        pars.require(XmlPullParser.END_TAG, null, TAG_GUARD);
        return g;
    }

    private static void loadGuardElement(XmlPullParser pars, Guard g) throws XmlPullParserException, IOException {
        pars.require(XmlPullParser.START_TAG, null, TAG_GUARD_ELEM);
        String sensor = pars.getAttributeValue(null, PROP_SENSOR);
        String op = pars.getAttributeValue(null, PROP_OPERATOR);
        int val = Integer.parseInt(pars.getAttributeValue(null, PROP_VALUE));
        g.add(sensor, op, val);
        pars.nextTag();
        pars.require(XmlPullParser.END_TAG, null, TAG_GUARD_ELEM);
    }

    private static BezierPath loadPath(XmlPullParser pars) throws IOException, XmlPullParserException {
        pars.require(XmlPullParser.START_TAG, null, TAG_PATH);

        List<PointF> points = new LinkedList<PointF>();

        while (pars.next() != XmlPullParser.END_TAG) {
            if (pars.getEventType() != XmlPullParser.START_TAG) {
                continue;
            }
            String tag = pars.getName();
            // Starts by looking for the entry tag
            if (tag.equals(TAG_POINT)) {
                points.add(loadPoint(pars));
            } else {
                skip(pars);
            }
        }

        pars.require(XmlPullParser.END_TAG, null, TAG_PATH);

        return new BezierPath(points);
    }

    private static PointF loadPoint(XmlPullParser pars) throws XmlPullParserException, IOException {
        pars.require(XmlPullParser.START_TAG, null, TAG_POINT);

        float x = Float.parseFloat(pars.getAttributeValue(null, PROP_X));
        float y = Float.parseFloat(pars.getAttributeValue(null, PROP_Y));

        pars.nextTag();
        pars.require(XmlPullParser.END_TAG, null, TAG_POINT);

        return new PointF(x, y);
    }

    private static void skip(XmlPullParser parser) throws XmlPullParserException, IOException {
        if (parser.getEventType() != XmlPullParser.START_TAG) {
            throw new IllegalStateException();
        }
        int depth = 1;
        while (depth != 0) {
            switch (parser.next()) {
                case XmlPullParser.END_TAG:
                    depth--;
                    break;
                case XmlPullParser.START_TAG:
                    depth++;
                    break;
            }
        }
    }
}
