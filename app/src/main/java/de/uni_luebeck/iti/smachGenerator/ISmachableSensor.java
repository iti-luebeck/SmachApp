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
package de.uni_luebeck.iti.smachGenerator;

import java.util.Set;

public interface ISmachableSensor {

    /**
     * @return the name of the sensor
     */
    public abstract String getName();

    /**
     * @return a HashSet of all Imports that are needed for this
     * {@link ISmachableSensor}.
     */
    public abstract Set<String> getImports();

    /**
     * Creates the callback function for this sensor. All calculations (if
     * necessary) to be able to compare values for later transitions have to be
     * done here and the result has to be stored in the
     * <code>ValueIdentifier</code>. This <code>ValueIdentifier</code> has to be
     * the same that is initialized by <code>getIdentifierInit()</code> and
     * <code>getGlobalIdentifier</code>.
     *
     * @return the callback for this sensor.
     */
    public abstract String getCallback();

    /**
     * Creates the Subscriber Setup statement. The callback for this Subscriber
     * will be the one that is created by <code>getCallback()</code>.
     *
     * @return Subscriber initialization
     */
    public abstract String getSubscriberSetup();

    /**
     * @return the identifier that stores the current value of this sensor.
     */
    public abstract String getValueIdentifier();

    /**
     * Returns the identifier of the globally defined variable for this sensor.
     * <p>
     * NOTE: this means this might return the name of an Array or anything else.
     * To receive the identifier that stores the current value of this sensor,
     * use <code>getValueIdentifier()</code>!</p>
     * <p><i>May not be empty!</i></p>
     *
     * @return globally identifier name.
     */
    public abstract String getGlobalIdentifier();

    /**
     * <p><i>May not be empty!</i></p>
     *
     * @return statements to define and initialize the value the callback stores
     * values in.
     */
    public abstract String getIdentifierInit();

    /**
     * Returns the condition comparing the current sensor value with the compare
     * value.
     *
     * @param op      Operator to compare the values. Choose one of <, <=, ==, >=, >, !=
     * @param compVal value to compare the current sensor value with.
     * @return a condition representing the comparison of the sensor value with
     * the compare value.
     */
    public abstract String getTransitionCondition(String op, int compVal);

    /**
     * Returns a number of commands that shall be executed before the {@link SmachAutomat} is shut down.
     * Mainly this will be publishing some last messages for the sensor to deactivate etc.
     *
     * @return Some commands to shutdown this sensor
     */
    public abstract String[] onShutDown();

    public String getTopic();

    public String getTopicType();

}
