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

import java.util.LinkedList;
import java.util.List;

import beep_msgs.Color;

/**
 * Created by Morten Mey on 04.07.2014.
 */
public class DebugModel {
    private EditorModel model;

    private State currentState;
    private List<Transition> lastTransitions=new LinkedList<Transition>();

    private short[] ir = new short[8];
    private int[] groundColors = new int[3];

    private List<DebugModelObserver> observers = new LinkedList<DebugModelObserver>();

    private boolean isRunning=false;
    private String ip;

    public DebugModel(EditorModel model,String ip) {
        this.model = model;
        this.ip=ip;

        for (State s : model.getStateMachine()) {
            if (s.isInitialState()) {
                currentState = s;
                break;
            }
        }
    }

    public EditorModel getEditor() {
        return model;
    }

    public void updateCurrentState(String state) {
        if(!state.equals(currentState.getName())) {
            lastTransitions.clear();
        }
        for(Transition t:currentState){
            if(t.getFollowerState().getName().equals(state)){
                lastTransitions.add(t);
                currentState=t.getFollowerState();
            }
        }

        notifyObservers();
    }

    public void updateIR(short[] ir) {
        if (ir.length != 8) {
            throw new IllegalArgumentException();
        }
        this.ir = ir;
        notifyObservers();
    }

    public void updateGroundColors(List<Color> colors) {
        if (colors.size() != 3) {
            //System.out.println(colors.size());
            return;
        }
        groundColors[0]=colorToInt(colors.get(0));
        groundColors[1]=colorToInt(colors.get(1));
        groundColors[2]=colorToInt(colors.get(2));
        notifyObservers();
    }

    private void notifyObservers() {
        for (DebugModelObserver obs : observers) {
            obs.onModelChange(this);
        }
    }

    public boolean addObserver(DebugModelObserver obs) {
        return observers.add(obs);
    }

    public boolean removeObserver(DebugModelObserver obs) {
        return observers.remove(obs);
    }

    public State getCurrentState() {
        return currentState;
    }

    public List<Transition> getLastTransitions(){
        return lastTransitions;
    }

    public short[] getIr() {
        return ir;
    }

    public int[] getGroundColors() {
        return groundColors;
    }

    public boolean isRunning(){
        return isRunning;
    }

    public void setRunning(boolean running){
        isRunning=running;
    }

    public String getIp(){
        return ip;
    }

    private static int colorToInt(Color col){
        return android.graphics.Color.argb(col.getW(),col.getR(),col.getG(),col.getB());
    }
}
