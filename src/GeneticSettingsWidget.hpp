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

#ifndef GENETIC_SETTINGS_WIDGET_H
#define GENETIC_SETTINGS_WIDGET_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QTimer>
#include <QPushButton>
#include "../third_party/qcustomplot/qcustomplot.h"

#include "Motor.hpp"
#include "Genetic.hpp"
#include "OddSpinBox.hpp"

class GeneticSettingsWidget : public QWidget {
    Q_OBJECT;

    public:
        GeneticSettingsWidget(Motor *mot, QCustomPlot *plot, QWidget *parent = 0);
        ~GeneticSettingsWidget();

    private slots:
        void start();
        void pause();
        void reset();
        void updatePop();
        void changeEvalTime(double time);
        void changeMutationRatio(double ratio);
        void changeCrossoverRatio(double ratio);

        void changeInput(double input);
        void changeMinKp(double kp);
        void changeMinKd(double kd);
        void changeMinKi(double ki);
        void changeMaxKp(double kp);
        void changeMaxKd(double kd);
        void changeMaxKi(double ki);
        void changePopulationSize(int);
        void changeOvershootPenalty(double);
        void changeEliteNum(int);

    private:
        void enableWidgets(bool enable);

        QDoubleSpinBox *m_inputDSpBx;
        QDoubleSpinBox *m_minKpDSpBx, *m_minKdDSpBx, *m_minKiDSpBx; // TODO change name DblSpBx
        QDoubleSpinBox *m_maxKpDSpBx, *m_maxKdDSpBx, *m_maxKiDSpBx;
        QDoubleSpinBox *m_evalTimeDSpBx;
        OddSpinBox *m_populationSizeSpBx;
        QDoubleSpinBox *m_mutationRatioDSpBx;
        QDoubleSpinBox *m_crossoverRatioDSpBx;
        QDoubleSpinBox *m_overshootPenaltyDSpBx;
        OddSpinBox *m_eliteNumSpBx;
        QPushButton *m_startBtn;
        QPushButton *m_pauseBtn;
        QPushButton *m_stopBtn;
        Genetic *m_genetic;
        Motor *m_motor;
        QCustomPlot *m_plot;
        QLabel *m_bestKpLbl, *m_bestKdLbl, *m_bestKiLbl, *m_iterationsLbl;
        QTimer *m_timer;
};
#endif
