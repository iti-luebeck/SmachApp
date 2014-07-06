package de.uni_luebeck.iti.smachapp.view;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Typeface;
import android.graphics.drawable.BitmapDrawable;
import android.util.AttributeSet;
import android.widget.ImageView;

import de.uni_luebeck.iti.smachapp.app.R;
import de.uni_luebeck.iti.smachapp.model.DebugModel;
import de.uni_luebeck.iti.smachapp.model.DebugModelObserver;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class SensorView extends ImageView implements DebugModelObserver {

    private DebugModel model;

    private float[] irX = new float[8];
    private float[] irY = new float[8];

    private float[] colorX = new float[3];
    private float[] colorY = new float[3];

    private Paint paint;
    private Bitmap image;
    private Canvas myCanvas;
    private Bitmap composedImage;

    public void setModel(DebugModel model) {
        this.model = model;
        model.addObserver(this);
    }

    public SensorView(Context context) {
        super(context);
        setup();
    }

    public SensorView(Context context, AttributeSet attrs) {
        super(context, attrs);
        setup();
    }

    public SensorView(Context context, AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
        setup();
    }

    private void setup() {
        paint = new Paint(Paint.ANTI_ALIAS_FLAG);
        paint.setTextSize(40);
        paint.setTypeface(Typeface.DEFAULT);

        irX[0] = 358 / 400f;
        irY[0] = 276 / 400f;
        irX[1] = 238 / 400f;
        irY[1] = 383 / 400f;
        irX[2] = 112 / 400f;
        irY[2] = 379 / 400f;
        irX[3] = 6 / 400f;
        irY[3] = 277 / 400f;
        irX[4] = 12 / 400f;
        irY[4] = 126 / 400f;
        irX[5] = 133 / 400f;
        irY[5] = 28 / 400f;
        irX[6] = 249 / 400f;
        irY[6] = 33 / 400f;
        irX[7] = 358 / 400f;
        irY[7] = 130 / 400f;

        colorX[0] = 141/400f;
        colorY[0] = 135/400f;
        colorX[1] = 192/400f;
        colorY[1] = 135/400f;
        colorX[2] = 243/400f;
        colorY[2] = 135/400f;

        image = BitmapFactory.decodeResource(getResources(), R.drawable.beep_debug_background);
        composedImage = Bitmap.createBitmap(image.getWidth(), image.getHeight(), Bitmap.Config.RGB_565);
        myCanvas = new Canvas(composedImage);
        this.setImageDrawable(new BitmapDrawable(getResources(), composedImage));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        myCanvas.drawBitmap(image, 0, 0, null);

        int width=getMeasuredWidth();
        int height=getMeasuredHeight();

        if (model != null) {
            short[] ir = model.getIr();

            for (int i = 0; i < irX.length; i++) {
                myCanvas.drawText("IR"+i+":"+Short.toString(ir[i]), irX[i]*width, irY[i]*height, paint);
            }

            int[] colors = model.getGroundColors();

            for (int i = 0; i < colorX.length; i++) {
                myCanvas.drawText(Integer.toString(colors[i]), colorX[i]*width, colorY[i]*height, paint);
            }
        }
        super.onDraw(canvas);
    }

    @Override
    public void onModelChange(DebugModel model) {
        postInvalidate();
    }
}
