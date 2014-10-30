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

#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QComboBox>
#include <QCheckBox>
#include <QDoubleSpinBox>

#include "../third_party/qcustomplot/qcustomplot.h"

#include "Motor.hpp"
#include "Controller.hpp"

class MainWindow : public QMainWindow {
    Q_OBJECT;
    public:
        MainWindow(QWidget *parent=0);

    private slots:
        void motorRun(void);

    private:
        QComboBox *m_motorCB;
        QDoubleSpinBox *m_runTimeDSpBx;
        QDoubleSpinBox *m_inputValueDSpBx;
        QComboBox *m_controllerCB;
        QCheckBox *m_closedLoopChkB;
        QCheckBox *m_useControllerChkB;
        QCustomPlot *m_plot;
        QVector<Motor*> m_motors;
        QVector<Controller*> m_controllers;
        QVector<QWidget*> m_controllerSettingsWidget;
};

#endif
