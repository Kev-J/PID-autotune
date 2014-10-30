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

#ifndef MOTOR_H
#define MOTOR_H

#include <QVector>
#include <QString>

#include "Controller.hpp"

class Motor {
    public:
        Motor(void);
        void setController(Controller *controller);
        void setVoltage(double input);
        virtual void runOpenedLoop(QVector<double> &outputValues, double input, double time) = 0;
        virtual void runClosedLoop(QVector<double> &outputValues, double input, double time) = 0;
        virtual double getSamplingPeriod(void) = 0;
        QString getName(void);

    protected:
        QString m_name;
        Controller *m_controller;

    private:
        double m_input;
};

#endif
