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

#include <QDockWidget>
#include <QVBoxLayout>
#include <iostream>
#include <QPushButton>
#include <QMenuBar>
#include <assert.h>

#include "MainWindow.hpp"
#include "DummyMotor.hpp"
#include "PIDController.hpp"
#include "PIDSettingWidget.hpp"
#include "GeneticSettingsWidget.hpp"
#include "GraphSettingsWidget.hpp"

MainWindow::MainWindow(QWidget *parent)
:QMainWindow(parent)
{
    setGeometry(0,0,800, 600);

    // Motors
    m_motors.push_back(new DummyMotor);

    // Controllers
    PIDController *pidController = new PIDController;
    m_controllers.push_back(pidController);

    // Controller settings widget
    m_controllerSettingsWidget.push_back(new PIDSettingWidget(pidController));

    // Widgets
    m_plot = new QCustomPlot();
    setCentralWidget(m_plot);

    //Genetic settings widget
    QDockWidget *geneticDockWidget = new QDockWidget("Genetic");
    geneticDockWidget->setWidget(new GeneticSettingsWidget(m_motors[0], m_plot));
    addDockWidget(Qt::LeftDockWidgetArea, geneticDockWidget);

    // Motor widget
    QDockWidget *motorDockWidget = new QDockWidget("Motor");
    QWidget *motorWidget = new QWidget;
    addDockWidget(Qt::RightDockWidgetArea, motorDockWidget);

    // Graph settings widget
    QDockWidget *graphDockWidget = new QDockWidget("Graph settings");
    graphDockWidget->setWidget(new GraphSettingsWidget(m_plot));
    addDockWidget(Qt::TopDockWidgetArea, graphDockWidget);

    QVBoxLayout *motorLayout = new QVBoxLayout;

    m_motorCB = new QComboBox;
    for (int i = 0 ; i < m_motors.size() ; i++) {
        m_motorCB->addItem(m_motors[i]->getName());
    }
    motorLayout->addWidget(m_motorCB);

    m_runTimeDSpBx = new QDoubleSpinBox;
    m_runTimeDSpBx->setPrefix("Running time : ");
    m_runTimeDSpBx->setSuffix("s");
    motorLayout->addWidget(m_runTimeDSpBx);

    m_inputValueDSpBx = new QDoubleSpinBox;
    m_inputValueDSpBx->setPrefix("Input : ");
    motorLayout->addWidget(m_inputValueDSpBx);

    QPushButton *runBtn = new QPushButton("Run");
    motorLayout->addWidget(runBtn);

    m_closedLoopChkB = new QCheckBox("Closed loop");
    motorLayout->addWidget(m_closedLoopChkB);

    motorWidget->setLayout(motorLayout);
    motorDockWidget->setWidget(motorWidget);

    // Controller widget
    QDockWidget *controllerDockWidget = new QDockWidget("Controller");
    QWidget *controllerWidget = new QWidget;
    addDockWidget(Qt::RightDockWidgetArea, controllerDockWidget);

    QVBoxLayout *controllerLayout = new QVBoxLayout;

    m_controllerCB = new QComboBox();
    for (int i = 0 ; i < m_controllers.size() ; i++) {
        m_controllerCB->addItem(m_controllers[i]->getName());
    }
    controllerLayout->addWidget(m_controllerCB);

    m_useControllerChkB = new QCheckBox("Use controller");
    controllerLayout->addWidget(m_useControllerChkB);

    controllerLayout->addWidget(m_controllerSettingsWidget[0]);

    controllerWidget->setLayout(controllerLayout);
    controllerDockWidget->setWidget(controllerWidget);

    m_plot->addGraph();

    connect(runBtn, SIGNAL(pressed()), this, SLOT(motorRun()));

    // Menu
    QMenu *viewMenu = menuBar()->addMenu("View");

    QAction *geneticAction = viewMenu->addAction("Genetic");
    geneticAction->setCheckable(true);
    geneticAction->setChecked(true);
    connect(geneticAction, SIGNAL(toggled(bool)), geneticDockWidget, SLOT(setVisible(bool)));
    connect(geneticDockWidget, SIGNAL(visibilityChanged(bool)), geneticAction, SLOT(setChecked(bool)));

    QAction *graphAction = viewMenu->addAction("Graph settings");
    graphAction->setCheckable(true);
    graphAction->setChecked(true);
    connect(graphAction, SIGNAL(toggled(bool)), graphDockWidget, SLOT(setVisible(bool)));
    connect(graphDockWidget, SIGNAL(visibilityChanged(bool)), graphAction, SLOT(setChecked(bool)));

    QAction *motorAction = viewMenu->addAction("Motor");
    motorAction->setCheckable(true);
    motorAction->setChecked(true);
    connect(motorAction, SIGNAL(toggled(bool)), motorDockWidget, SLOT(setVisible(bool)));
    connect(motorDockWidget, SIGNAL(visibilityChanged(bool)), motorAction, SLOT(setChecked(bool)));

    QAction *controllerAction = viewMenu->addAction("Controller");
    controllerAction->setCheckable(true);
    controllerAction->setChecked(true);
    connect(controllerAction, SIGNAL(toggled(bool)), controllerDockWidget, SLOT(setVisible(bool)));
    connect(controllerDockWidget, SIGNAL(visibilityChanged(bool)), controllerAction, SLOT(setChecked(bool)));
}

void MainWindow::motorRun(void)
{
    double time = m_runTimeDSpBx->value();

    assert((m_motorCB->currentIndex() > -1) && (m_motorCB->currentIndex() < m_motors.size()));

    Motor *currMotor = m_motors[m_motorCB->currentIndex()];
    double samplingPer = currMotor->getSamplingPeriod();
    int nbSamples = (int)(time / samplingPer);

    QVector<double> x(nbSamples), y(nbSamples);

    if (m_useControllerChkB->isChecked()) {
        currMotor->setController(m_controllers[m_controllerCB->currentIndex()]);
        m_controllers[m_controllerCB->currentIndex()]->reset();
    } else {
        currMotor->setController(0);
    }

    if (m_closedLoopChkB->isChecked()) {
        currMotor->runClosedLoop(y, m_inputValueDSpBx->value(), time);
    } else {
        currMotor->runOpenedLoop(y, m_inputValueDSpBx->value(), time);
    }

    double t = 0.0;
    for (int i = 0 ; i < nbSamples ; i++) {
        x[i] = t;
        t += samplingPer;
    }

    m_plot->graph(0)->setData(x,y);
    m_plot->replot();
}
