package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.content.SharedPreferences;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.PointF;
import android.graphics.Rect;
import android.graphics.RectF;
import android.graphics.Typeface;
import android.preference.PreferenceManager;
import android.util.AttributeSet;
import android.view.View;

import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.EditorModel;
import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;
import de.uni_luebeck.iti.smachapp.utils.RectUtils;

/**
 * Created by Morten Mey on 27.04.2014.
 */
public class StateMachineView extends View {

    private static final float INITIAL_STATE_OFFSET = 10f;
    private static final float STATE_OVAL_OFFSET = 25f;
    private static final float DEBUG_POINT_SIZE = 4f;

    private static Rect helperRect = new Rect();

    private EditorModel model = null;

    private float scale = 1f;
    private float translationX = 0f;
    private float translationY = 0f;

    private Paint paint;
    private Paint textPaint;
    private Paint highlightPaint;
    private Paint highlightTextPaint;
    private Paint debugPaint;
    private Paint arrowPaint;
    private Paint transitionTextPaint;

    private RectF rect = new RectF();
    private Rect clipBounds = new Rect();

    private Path path = new Path();
    private Path tempPath;

    private List<Transition> highlightedTransitions = new LinkedList<Transition>();

    private List<State> highlightedStates = new LinkedList<State>();

    private Path arrowHead = new Path();

    private static StateMachineView currentView;

    public static StateMachineView getCurrentView() {
        return currentView;
    }

    public StateMachineView(Context context) {
        super(context);
        setup();
    }

    public StateMachineView(Context context, AttributeSet attrs) {
        super(context, attrs);
        setup();
    }

    public StateMachineView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        setup();
    }

    private void setup() {
        currentView = this;
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(4);

        highlightPaint = new Paint(paint);
        highlightPaint.setColor(Color.BLUE);

        textPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        textPaint.setTextAlign(Paint.Align.CENTER);
        textPaint.setTextSize(40);
        textPaint.setTypeface(Typeface.DEFAULT);

        transitionTextPaint = new Paint(textPaint);
        transitionTextPaint.setTextAlign(Paint.Align.RIGHT);

        highlightTextPaint = new Paint(textPaint);
        highlightTextPaint.setColor(Color.BLUE);

        debugPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
        debugPaint.setColor(Color.RED);

        arrowPaint = new Paint(Paint.ANTI_ALIAS_FLAG);
    }

    public void setModel(EditorModel model) {
        this.model = model;
    }

    @Override
    protected void onDraw(Canvas canvas) {

        int width = getWidth() / 2;
        int height = getHeight() / 2;

        canvas.scale(scale, scale, width, height);

        canvas.translate(translationX + width, translationY + height);
        canvas.getClipBounds(clipBounds);

        SharedPreferences sharedPref = PreferenceManager.getDefaultSharedPreferences(getContext());
        boolean drawTransitionNames = sharedPref.getBoolean(getContext().getString(R.string.preference_key_showTransitionNames), false);

        if (model == null) {
            return;
        }
        for (State state : model.getStateMachine()) {
            Paint oval, text;

            for (Transition trans : state) {
                if (highlightedTransitions.contains(trans)) {
                    oval = highlightPaint;
                } else {
                    oval = paint;
                }

                path.rewind();
                trans.getPath().fillPath(path);
                canvas.drawPath(path, oval);

                List<PointF> points = trans.getPath().getPoints();
                makeArrowHead(trans.getPath().calculatePointOnBezier(-1, 0.95f), points.get(points.size() - 1));
                canvas.drawPath(arrowHead, arrowPaint);

                boolean drawBezierKnots = true;
                if (drawBezierKnots) {
                    for (PointF point : trans.getPath().getPoints()) {
                        RectUtils.makeRectFromPoint(point, rect);
                        RectUtils.extendRect(rect, DEBUG_POINT_SIZE);
                        canvas.drawOval(rect, debugPaint);
                    }
                }

                if (drawTransitionNames) {
                    PointF point = trans.getPath().calculatePointOnBezier((points.size() - 1) / 2, 0.5f);
                    canvas.drawText(trans.getLabel(), point.x - 20, point.y - 20, transitionTextPaint);
                }
            }

            if (highlightedStates.contains(state)) {
                oval = highlightPaint;
                text = highlightTextPaint;
            } else {
                oval = paint;
                text = textPaint;
            }
            getStateRect(state, rect, true);
            canvas.drawOval(rect, oval);
            canvas.drawText(state.getName(), state.getX(), state.getY(), text);

            if (state.isInitialState()) {
                RectUtils.extendRect(rect, INITIAL_STATE_OFFSET);
                canvas.drawOval(rect, oval);
            }
        }

        if (tempPath != null) {
            canvas.drawPath(tempPath, highlightPaint);
        }
    }

    public void setScale(float scale) {
        this.scale = scale;
    }

    public float getScale() {
        return scale;
    }

    public void setTranslationX(float x) {
        translationX = x;
    }

    public void setTranslationY(float y) {
        translationY = y;
    }

    public float getTranslationX() {
        return translationX;
    }

    public float getTranslationY() {
        return translationY;
    }

    public void scale(float factor) {
        scale *= factor;
        postInvalidate();
    }

    public void translate(float x, float y) {
        translationX -= x / scale;
        translationY -= y / scale;
        postInvalidate();
    }

    public void reset() {
        scale = 1f;
        translationX = 0;
        translationY = 0;
        postInvalidate();
    }

    public void getStateRect(State state, RectF result) {
        this.getStateRect(state, result, false);
    }

    public void getStateRect(State state, RectF result, boolean forDrawing) {
        textPaint.getTextBounds(state.getName(), 0, state.getName().length(), helperRect);

        int width = helperRect.width() / 2;
        int height = helperRect.height() / 2;

        result.left = state.getX() - STATE_OVAL_OFFSET - width;
        result.right = state.getX() + STATE_OVAL_OFFSET + width;
        result.top = state.getY() - STATE_OVAL_OFFSET - height;
        result.bottom = state.getY() + STATE_OVAL_OFFSET - height;

        if (!forDrawing && state.isInitialState()) {
            RectUtils.extendRect(result, INITIAL_STATE_OFFSET);
        }
    }

    public void translatePoint(PointF point) {
        point.x = point.x / scale + clipBounds.left;
        point.y = point.y / scale + clipBounds.top;
    }

    public void highlighteStates(List<State> s) {
        highlightedStates = s;
        postInvalidate();
    }

    public void highlighteTransitions(List<Transition> trans) {
        highlightedTransitions = trans;
        postInvalidate();
    }

    public void setTempPath(Path p) {
        tempPath = p;
        postInvalidate();
    }

    private void makeArrowHead(PointF secondPoint, PointF endPoint) {

        PointF dir = PointUtils.calculateDirection(endPoint, secondPoint);
        PointUtils.normalize(dir);

        //noinspection SuspiciousNameCombination
        PointF first = new PointF(-dir.y, dir.x);
        PointF second = new PointF(dir.y, -dir.x);
        first.x *= 10;
        first.y *= 10;
        second.x *= 10;
        second.y *= 10;
        dir.x *= 20;
        dir.y *= 20;
        dir.x += endPoint.x;
        dir.y += endPoint.y;

        arrowHead.rewind();
        arrowHead.moveTo(endPoint.x, endPoint.y);
        arrowHead.lineTo(dir.x + first.x, dir.y + first.y);
        arrowHead.lineTo(dir.x + second.x, dir.y + second.y);
        arrowHead.close();
    }
}
