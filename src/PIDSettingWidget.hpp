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

#ifndef PIDSettingWidget_H
#define PIDSettingWidget_H

#include <QWidget>

#include "PIDController.hpp"

class PIDSettingWidget : public QWidget {
    Q_OBJECT;
    public:
        PIDSettingWidget(PIDController *controller, QWidget *parent = 0);

    private slots:
        void changeKp(double kp);
        void changeKd(double kd);
        void changeKi(double ki);

    private:
        PIDController *m_controller;
};

#endif
