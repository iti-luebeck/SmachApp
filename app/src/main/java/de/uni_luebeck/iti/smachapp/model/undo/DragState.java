/*
 * Copyright (c) 2015, Institute of Computer Engineering, University of Lübeck
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * 
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * 
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package de.uni_luebeck.iti.smachapp.model.undo;

import android.graphics.PointF;

import java.util.Hashtable;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DragState implements UndoableAction {

    private State state;
    private List<Transition> transitions;

    private float xBefore, yBefore;
    private float xAfter, yAfter;
    private Hashtable<Transition, List<PointF>> before = new Hashtable<Transition, List<PointF>>();
    private Hashtable<Transition, List<PointF>> after = new Hashtable<Transition, List<PointF>>();


    public DragState(State state, List<Transition> incomingTransitions) {
        this.state = state;
        this.transitions = new LinkedList<Transition>(incomingTransitions);
        transitions.addAll(state.getTransitions());
        xBefore = state.getX();
        yBefore = state.getY();

        for (Transition t : transitions) {
            before.put(t, PointUtils.copyPoints(t.getPath().getPoints()));
        }
    }

    public void operationDone() {
        xAfter = state.getX();
        yAfter = state.getY();

        for (Transition t : transitions) {
            after.put(t, PointUtils.copyPoints(t.getPath().getPoints()));
        }
    }

    @Override
    public void undo() {
        state.setX(xBefore);
        state.setY(yBefore);

        for (Transition t : transitions) {
            t.getPath().getPoints().clear();
            t.getPath().getPoints().addAll(PointUtils.copyPoints(before.get(t)));
            t.getPath().resize();
        }
    }

    @Override
    public void redo() {
        state.setX(xAfter);
        state.setY(yAfter);

        for (Transition t : transitions) {
            t.getPath().getPoints().clear();
            t.getPath().getPoints().addAll(PointUtils.copyPoints(after.get(t)));
            t.getPath().resize();
        }
    }
}
