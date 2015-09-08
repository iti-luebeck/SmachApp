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

import java.util.List;

import de.uni_luebeck.iti.smachapp.model.State;
import de.uni_luebeck.iti.smachapp.model.Transition;
import de.uni_luebeck.iti.smachapp.utils.PointUtils;

/**
 * Created by Morten Mey on 29.06.2014.
 */
public class DragTransition implements UndoableAction {

    private Transition transition;

    private State prevBefore;
    private State nextBefore;

    private State prefAfter;
    private State nextAfter;

    private List<PointF> before;
    private List<PointF> after;

    public DragTransition(Transition transition) {
        this.transition = transition;
        prevBefore = transition.getPreviousState();
        nextBefore = transition.getFollowerState();
        before = PointUtils.copyPoints(transition.getPath().getPoints());
    }


    public void operationDone() {
        prefAfter = transition.getPreviousState();
        nextAfter = transition.getFollowerState();
        after = PointUtils.copyPoints(transition.getPath().getPoints());
    }

    @Override
    public void undo() {
        transition.setFollowerState(nextBefore);
        State prev=transition.getPreviousState();
        if(prev !=null){
            prev.removeTransition(transition);
        }
        transition.setPreviousState(prevBefore);
        prevBefore.addTransition(transition);
        transition.getPath().getPoints().clear();
        transition.getPath().getPoints().addAll(PointUtils.copyPoints(before));
        transition.getPath().resize();
    }

    @Override
    public void redo() {
        State prev=transition.getPreviousState();
        if(prev !=null) {
            prev.removeTransition(transition);
        }
        transition.setPreviousState(prefAfter);
        prefAfter.addTransition(transition);
        transition.setFollowerState(nextAfter);
        transition.getPath().getPoints().clear();
        transition.getPath().getPoints().addAll(PointUtils.copyPoints(after));
        transition.getPath().resize();
    }
}
