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
package de.uni_luebeck.iti.smachapp.model;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import de.uni_luebeck.iti.smachGenerator.ISmachableState;

/**
 * Created by Morten Mey on 21.04.2014.
 */
public class State implements ISmachableState, Iterable<Transition> {

    private String name;
    private boolean initialState;
    private LinkedList<Transition> transitions = new LinkedList<Transition>();
    private LinkedList<Action> actions = new LinkedList<Action>();
    private LinkedList<Action> disabledActions=new LinkedList<Action>();

    private float x, y;

    public State(String name, float x, float y) {
        this(name, x, y, false);
    }

    public State(String name, float x, float y, boolean initialState) {
        this.name = name;
        this.initialState = initialState;
        this.x = x;
        this.y = y;
    }

    @Override
    public List<Action> getActions() {
        return actions;
    }

    @Override
    public List<Transition> getTransitions() {
        return transitions;
    }

    @Override
    public boolean isInitialState() {
        return initialState;
    }

    @Override
    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public void addTransition(Transition t) {
        if (t.getPreviousState() != this) {
            throw new IllegalArgumentException("This is not the previous state.");
        }
        transitions.add(t);
    }

    public void removeTransition(Transition t) {
        transitions.remove(t);
    }

    public Iterator<Transition> transitionIterator() {
        return transitions.iterator();
    }

    public void addAction(Action a) {
        actions.add(a);
    }

    public void removeAction(Action a) {
        actions.remove(a);
    }

    public Iterator<Action> actionIterator() {
        return actions.iterator();
    }

    public float getX() {
        return x;
    }

    public float getY() {
        return y;
    }

    public void setX(float x) {
        this.x = x;
    }

    public void setY(float y) {
        this.y = y;
    }

    public void setCenter(float x, float y) {
        this.x = x;
        this.y = y;
    }

    @Override
    public Iterator<Transition> iterator() {
        return transitions.iterator();
    }

    public void setInitialState(boolean init) {
        initialState = init;
    }

    public List<Action> getDisabledActions(){
        return disabledActions;
    }
}
