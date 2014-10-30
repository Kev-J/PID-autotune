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

#include <QGridLayout>
#include <QLabel>
#include <QCoreApplication>
#include <QSpinBox>
#include <QMessageBox>
#include <iostream>

#include "GeneticSettingsWidget.hpp"

GeneticSettingsWidget::GeneticSettingsWidget(Motor *mot, QCustomPlot *plot, QWidget *parent)
:QWidget(parent), m_motor(mot), m_plot(plot)
{
    m_genetic = new Genetic(mot);
    m_genetic->setPopulationSize(20);
    m_genetic->generatePopulation();

    m_timer = new QTimer;
    m_timer->setInterval(0);
    connect(m_timer, SIGNAL(timeout()), this, SLOT(updatePop()));

    QGridLayout *layout = new QGridLayout();

    // Input spinbox
    m_inputDSpBx = new QDoubleSpinBox();
    m_inputDSpBx->setPrefix("Input : ");
    m_inputDSpBx->setValue(m_genetic->getInput());
    m_inputDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_inputDSpBx, 0, 0, 1, 2);
    connect(m_inputDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeInput(double)));

    // min/max PID parameter spinboxes
    m_minKpDSpBx = new QDoubleSpinBox();
    m_minKpDSpBx->setPrefix("min Kp : ");
    m_minKpDSpBx->setDecimals(5);
    m_minKpDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_minKpDSpBx, 1, 0);
    connect(m_minKpDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMinKp(double)));

    m_minKdDSpBx = new QDoubleSpinBox();
    m_minKdDSpBx->setPrefix("min Kd : ");
    m_minKdDSpBx->setDecimals(5);
    m_minKdDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_minKdDSpBx, 2, 0);
    connect(m_minKdDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMinKd(double)));

    m_minKiDSpBx = new QDoubleSpinBox();
    m_minKiDSpBx->setPrefix("min Ki : ");
    m_minKiDSpBx->setDecimals(5);
    m_minKiDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_minKiDSpBx, 3, 0);
    connect(m_minKiDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMinKi(double)));

    m_maxKpDSpBx = new QDoubleSpinBox();
    m_maxKpDSpBx->setPrefix("max Kp : ");
    m_maxKpDSpBx->setDecimals(5);
    m_maxKpDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_maxKpDSpBx, 1, 1);
    connect(m_maxKpDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMaxKp(double)));

    m_maxKdDSpBx = new QDoubleSpinBox();
    m_maxKdDSpBx->setPrefix("max Kd : ");
    m_maxKdDSpBx->setDecimals(5);
    m_maxKdDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_maxKdDSpBx, 2, 1);
    connect(m_maxKdDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMaxKd(double)));

    m_maxKiDSpBx = new QDoubleSpinBox();
    m_maxKiDSpBx->setPrefix("max Ki : ");
    m_maxKiDSpBx->setDecimals(5);
    m_maxKiDSpBx->setRange(0.0, 100000.0);
    layout->addWidget(m_maxKiDSpBx, 3, 1);
    connect(m_maxKiDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMaxKi(double)));

    // Evaluation time spinbox 
    m_evalTimeDSpBx = new QDoubleSpinBox;
    m_evalTimeDSpBx->setPrefix("Evaluation time : ");
    m_evalTimeDSpBx->setDecimals(5);
    m_evalTimeDSpBx->setRange(0.0, 10000.0);
    layout->addWidget(m_evalTimeDSpBx, 4, 0, 1, 2);
    m_evalTimeDSpBx->setValue(m_genetic->getEvalTime());
    connect(m_evalTimeDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeEvalTime(double)));

    // Population size spinbox
    m_populationSizeSpBx = new OddSpinBox;
    m_populationSizeSpBx->setPrefix("Population size : ");
    m_populationSizeSpBx->setSingleStep(2);
    m_populationSizeSpBx->setRange(4, 100000);
    m_populationSizeSpBx->setValue(m_genetic->getPopulationSize());
    layout->addWidget(m_populationSizeSpBx, 5, 0, 1, 2);
    connect(m_populationSizeSpBx, SIGNAL(valueChanged(int)), this, SLOT(changePopulationSize(int)));

    // Mutation ratio spinbox
    m_mutationRatioDSpBx = new QDoubleSpinBox;
    m_mutationRatioDSpBx->setPrefix("Mutation ratio : ");
    m_mutationRatioDSpBx->setRange(0.0, 1.0);
    m_mutationRatioDSpBx->setSingleStep(0.001);
    m_mutationRatioDSpBx->setDecimals(5);
    m_mutationRatioDSpBx->setValue(m_genetic->getMutationRatio());
    layout->addWidget(m_mutationRatioDSpBx, 6, 0, 1, 2);
    connect(m_mutationRatioDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeMutationRatio(double)));

    // Crossover ratio spinbox
    m_crossoverRatioDSpBx = new QDoubleSpinBox;
    m_crossoverRatioDSpBx->setPrefix("Crossover ratio : ");
    m_crossoverRatioDSpBx->setRange(0.0, 1.0);
    m_crossoverRatioDSpBx->setSingleStep(0.001);
    m_crossoverRatioDSpBx->setDecimals(5);
    m_crossoverRatioDSpBx->setValue(m_genetic->getCrossoverRatio());
    layout->addWidget(m_crossoverRatioDSpBx, 7, 0, 1, 2);
    connect(m_crossoverRatioDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeCrossoverRatio(double)));

    // Overshoot penalty spinbox
    m_overshootPenaltyDSpBx = new QDoubleSpinBox;
    m_overshootPenaltyDSpBx->setPrefix("Overshoot penalty : ");
    m_overshootPenaltyDSpBx->setDecimals(5);
    m_overshootPenaltyDSpBx->setRange(0.0, 1000000.0);
    m_overshootPenaltyDSpBx->setValue(m_genetic->getOvershootPenalty());
    layout->addWidget(m_overshootPenaltyDSpBx, 8, 0, 1, 2);
    connect(m_overshootPenaltyDSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeOvershootPenalty(double)));

    // Elite num
    m_eliteNumSpBx = new OddSpinBox;
    m_eliteNumSpBx->setPrefix("Elite num : ");
    m_eliteNumSpBx->setRange(0, 100000);
    m_eliteNumSpBx->setValue(m_genetic->getEliteNum());
    layout->addWidget(m_eliteNumSpBx, 9, 0, 1, 2);
    connect(m_eliteNumSpBx, SIGNAL(valueChanged(int)), this, SLOT(changeEliteNum(int)));

    m_startBtn = new QPushButton("Start");
    layout->addWidget(m_startBtn, 10, 0, 1, 2);
    connect(m_startBtn, SIGNAL(pressed()), this, SLOT(start()));

    m_pauseBtn = new QPushButton("Pause");
    m_pauseBtn->setEnabled(false);
    layout->addWidget(m_pauseBtn, 11, 0);
    connect(m_pauseBtn, SIGNAL(pressed()), this, SLOT(pause()));

    m_stopBtn = new QPushButton("Reset");
    layout->addWidget(m_stopBtn, 11, 1);
    connect(m_stopBtn, SIGNAL(pressed()), this, SLOT(reset()));

    layout->addWidget(new QLabel("Best PID: Kp = "), 12, 0);
    m_bestKpLbl = new QLabel("none");
    layout->addWidget(m_bestKpLbl, 12, 1);

    layout->addWidget(new QLabel("Best PID: Kd = "), 13, 0);
    m_bestKdLbl = new QLabel("none");
    layout->addWidget(m_bestKdLbl, 13, 1);

    layout->addWidget(new QLabel("Best PID: Ki = "), 14, 0);
    m_bestKiLbl = new QLabel("none");
    layout->addWidget(m_bestKiLbl, 14, 1);

    layout->addWidget(new QLabel("Iterations : "), 15, 0);
    m_iterationsLbl = new QLabel("0");
    layout->addWidget(m_iterationsLbl, 15, 1);

    setLayout(layout);


}

GeneticSettingsWidget::~GeneticSettingsWidget()
{
    delete m_genetic;
}

void GeneticSettingsWidget::updatePop()
{
    m_genetic->updatePopulation();
    PIDController controller = m_genetic->getBestController();

    m_bestKpLbl->setText(QString::number(controller.getKp()));
    m_bestKdLbl->setText(QString::number(controller.getKd()));
    m_bestKiLbl->setText(QString::number(controller.getKi()));
    m_iterationsLbl->setText(QString::number(m_genetic->getIterations()));

    double time = m_genetic->getEvalTime();
    double samplingPer = m_motor->getSamplingPeriod();
    int nbSamples = (int)(time / samplingPer);

    QVector<double> x(nbSamples), y(nbSamples);

    m_motor->setController(&controller);

    m_motor->runClosedLoop(y, m_genetic->getInput(), time);

    double t = 0.0;
    for (int i = 0 ; i < nbSamples ; i++) {
        x[i] = t;
        t += samplingPer;
    }

    m_plot->graph(0)->setData(x,y);
    m_plot->replot();
    QCoreApplication::processEvents();
}

void GeneticSettingsWidget::changeEvalTime(double time)
{
    m_genetic->setEvalTime(time);
}

void GeneticSettingsWidget::changeMutationRatio(double ratio)
{
    m_genetic->setMutationRatio(ratio);
}

void GeneticSettingsWidget::changeCrossoverRatio(double ratio)
{
    m_genetic->setCrossoverRatio(ratio);
}

void GeneticSettingsWidget::changeInput(double input)
{
    m_genetic->setInput(input);
}

void GeneticSettingsWidget::changeMinKp(double kp)
{
    if (kp > m_maxKpDSpBx->value()) {
        kp = m_maxKpDSpBx->value();
        m_minKpDSpBx->setValue(kp);
    }

    m_genetic->setMinKp(kp);
}

void GeneticSettingsWidget::changeMinKd(double kd)
{
    if (kd > m_maxKdDSpBx->value()) {
        kd = m_maxKdDSpBx->value();
        m_minKdDSpBx->setValue(kd);
    }

    m_genetic->setMinKd(kd);
}

void GeneticSettingsWidget::changeMinKi(double ki)
{
    if (ki > m_maxKiDSpBx->value()) {
        ki = m_maxKiDSpBx->value();
        m_minKiDSpBx->setValue(ki);
    }

    m_genetic->setMinKi(ki);
}

void GeneticSettingsWidget::changeMaxKp(double kp)
{
    if (kp < m_minKpDSpBx->value()) {
        kp = m_minKpDSpBx->value();
        m_maxKpDSpBx->setValue(kp);
    }

    m_genetic->setMaxKp(kp);
}

void GeneticSettingsWidget::changeMaxKd(double kd)
{
    if (kd < m_minKdDSpBx->value()) {
        kd = m_minKdDSpBx->value();
        m_maxKdDSpBx->setValue(kd);
    }

    m_genetic->setMaxKd(kd);
}

void GeneticSettingsWidget::changeMaxKi(double ki)
{
    if (ki < m_minKiDSpBx->value()) {
        ki = m_minKiDSpBx->value();
        m_maxKiDSpBx->setValue(ki);
    }

    m_genetic->setMaxKi(ki);
}

void GeneticSettingsWidget::changePopulationSize(int size)
{
    if (size < m_eliteNumSpBx->value()) {
        size = m_eliteNumSpBx->value();
        m_populationSizeSpBx->setValue(size);
    }

    m_genetic->setPopulationSize(size);
}

void GeneticSettingsWidget::changeOvershootPenalty(double penalty)
{
    m_genetic->setOvershootPenalty(penalty);
}

void GeneticSettingsWidget::changeEliteNum(int num)
{
    if (num > m_populationSizeSpBx->value()) {
        num = m_populationSizeSpBx->value();
        m_eliteNumSpBx->setValue(num);
    }

    m_genetic->setEliteNum(num);
}

void GeneticSettingsWidget::start()
{
    m_timer->start();
    enableWidgets(false);
    m_pauseBtn->setEnabled(true);
    m_startBtn->setEnabled(false);
}

void GeneticSettingsWidget::pause()
{
    m_timer->stop();
    m_pauseBtn->setEnabled(false);
    m_startBtn->setEnabled(true);
}

void GeneticSettingsWidget::reset()
{
    if (QMessageBox::question(this, "Reset population", "You will reset the population. Continue?",
        QMessageBox::Yes|QMessageBox::No) == QMessageBox::Yes) {

        m_timer->stop();
        enableWidgets(true);
        m_genetic->generatePopulation();
        m_pauseBtn->setEnabled(false);
        m_startBtn->setEnabled(true);
    }
}

void GeneticSettingsWidget::enableWidgets(bool enable)
{
    m_inputDSpBx->setEnabled(enable);
    m_minKpDSpBx->setEnabled(enable); m_maxKpDSpBx->setEnabled(enable);
    m_minKdDSpBx->setEnabled(enable); m_maxKdDSpBx->setEnabled(enable);
    m_minKiDSpBx->setEnabled(enable); m_maxKiDSpBx->setEnabled(enable);
    m_evalTimeDSpBx->setEnabled(enable);
    m_populationSizeSpBx->setEnabled(enable);
    m_mutationRatioDSpBx->setEnabled(enable);
    m_crossoverRatioDSpBx->setEnabled(enable);
    m_overshootPenaltyDSpBx->setEnabled(enable);
    m_eliteNumSpBx->setEnabled(enable);
}
