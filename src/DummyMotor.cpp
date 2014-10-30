/*
 * Copyright (C) 2014 Kevin JOLY
 *
 * Authors :    Kevin JOLY <joly.kevin25@gmail.com>
 *
 * This file is part of PID-autotune.
 * 
 * PID-autotune is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * PID-autotune is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with PID-autotune.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "DummyMotor.hpp"

DummyMotor::DummyMotor()
:Motor(),
// Constants values found from Tustin methods applied on data from : 
// http://wikimeca.org/index.php?title=Moteur_%C3%A0_courant_continu
    K(2.5),
    AD(18.01), BD(-39.98),CD(22.01),
    MAX_OUTPUT(100.0),MIN_OUTPUT(-100.0),SAMPLING_TIME(0.001),
    C1(K/CD),C2(2.0*K/CD),C3(K/CD),C4(BD/CD),C5(AD/CD)
{
    m_name = "DummyMotor";
    reset();
}

void DummyMotor::reset()
{
    m_yn1 = 0.0;
    m_yn2 = 0.0;
    m_xn1 = 0.0;
    m_xn2 = 0.0;
    m_output = 0.0;
}

void DummyMotor::runOpenedLoop(QVector<double> &outputValues, double input, double time)
{
    int max = (int)(time / SAMPLING_TIME);
    double controllerOutput;

    reset();

    for (int i = 0; i < max ; i++) {
        if (m_controller != 0)
            controllerOutput = m_controller->sample(input);
        else
            controllerOutput = input;

        outputValues[i] = nextSample(controllerOutput);
    }
}

void DummyMotor::runClosedLoop(QVector<double> &outputValues, double input, double time)
{
    int max = (int)(time / SAMPLING_TIME);
    double error = input, controllerOutput;

    reset();

    for (int i = 0; i < max ; i++) {
        if (m_controller != 0)
            controllerOutput = m_controller->sample(error);
        else
            controllerOutput = error;

        outputValues[i] = nextSample(controllerOutput);
        error = input - outputValues[i];
    }
}

double DummyMotor::getSamplingPeriod(void)
{
    return SAMPLING_TIME;
}

// Equation found from Tustin method
double DummyMotor::nextSample(double input)
{
    m_output = C1*input;
    m_output += C2*m_xn1;
    m_output += C3*m_xn2;
    m_output -= C4*m_yn1;
    m_output -= C5*m_yn2;

    m_xn2 = m_xn1;
    m_xn1 = input;
    m_yn2 = m_yn1;
    m_yn1 = m_output;

    m_output = m_output < MAX_OUTPUT ? m_output : MAX_OUTPUT;
    m_output = m_output > MIN_OUTPUT ? m_output : MIN_OUTPUT;

    return m_output;
}
