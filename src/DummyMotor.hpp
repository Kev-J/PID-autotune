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

#ifndef DUMMY_MOTOR_H
#define DUMMY_MOTOR_H

#include "Motor.hpp"

class DummyMotor : public Motor {
    public:
        DummyMotor();
        void runOpenedLoop(QVector<double> &outputValues, double input, double time);
        void runClosedLoop(QVector<double> &outputValues, double input, double time);
        double getSamplingPeriod(void);

    private:
        // Methods
        void reset();
        double nextSample(double input);

        // Constants
        const double K;

        const double AD;
        const double BD;
        const double CD;

        const double MAX_OUTPUT;
        const double MIN_OUTPUT;
        const double SAMPLING_TIME;

        const double C1;
        const double C2;
        const double C3;
        const double C4;
        const double C5;

        // Attributes
        double m_yn1, m_yn2, m_xn1, m_xn2;
        double m_output;
};

#endif
