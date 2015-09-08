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

public interface ISmachableActuator {

    /**
     * @return the name of the sensor
     */
    public abstract String getName();

    /**
     * Creates the publisher setup statement. These definitions will be done
     * globally. The name of the publisher will be the same as
     * <code>getPublisherName()</code>.
     *
     * @return Publisher initialization
     */
    public abstract String getPublisherSetup();

    /**
     * @return the name of the publisher.
     */
    public abstract String getPublisherName();

    /**
     * Creates all statements that are needed to create a ros message for this
     * {@link ISmachableActuator} and to publish it. Add all statements in
     * ascending oder to publish the message.
     *
     * @param a {@link ISmachableAction} that represents the command and the
     *          content of the message.
     * @return a ordered amount of statements to publish a message for this
     * {@link ISmachableActuator}
     */
    public abstract String[] getPublishMessage(ISmachableAction a);

    /**
     * @return a HashSet of all Imports that are needed for this
     * {@link ISmachableActuator}.
     */
    public abstract Set<String> getImports();


    /**
     * Returns a number of commands that shall be executed before the {@link SmachAutomat} is shut down.
     * Mainly this will be publishing some last messages for the actuator to deactivate etc.
     *
     * @return Some commands to shutdown this actuator
     */
    public abstract String[] onShutDown();

}
