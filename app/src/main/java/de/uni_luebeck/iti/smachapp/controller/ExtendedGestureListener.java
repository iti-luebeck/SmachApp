package de.uni_luebeck.iti.smachapp.controller;

import android.view.GestureDetector;
import android.view.MenuItem;
import android.view.MotionEvent;

/**
 * Created by Morten Mey on 28.04.2014.
 */
public interface ExtendedGestureListener extends GestureDetector.OnGestureListener {

    public void onUp(MotionEvent e);

    boolean onContextItemSelected(MenuItem item);
}
