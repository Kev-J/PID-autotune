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

#include "PIDSettingWidget.hpp"

#include <QDoubleSpinBox>
#include <QGridLayout>
#include <QLabel>
#include <assert.h>

PIDSettingWidget::PIDSettingWidget(PIDController *controller, QWidget *parent)
:QWidget(parent)
{
    m_controller = controller;

    QDoubleSpinBox *kpDSB = new QDoubleSpinBox;
    QDoubleSpinBox *kiDSB = new QDoubleSpinBox;
    QDoubleSpinBox *kdDSB = new QDoubleSpinBox;

    kpDSB->setDecimals(5);
    kdDSB->setDecimals(5);
    kiDSB->setDecimals(5);

    QGridLayout *layout = new QGridLayout;

    layout->addWidget(kpDSB,1,1);
    layout->addWidget(kdDSB,2,1);
    layout->addWidget(kiDSB,3,1);

    layout->addWidget(new QLabel("Kp:"),1,0);
    layout->addWidget(new QLabel("Kd:"),2,0);
    layout->addWidget(new QLabel("Ki:"),3,0);

    connect(kpDSB, SIGNAL(valueChanged(double)), this, SLOT(changeKp(double)));
    connect(kdDSB, SIGNAL(valueChanged(double)), this, SLOT(changeKd(double)));
    connect(kiDSB, SIGNAL(valueChanged(double)), this, SLOT(changeKi(double)));

    this->setLayout(layout);
}

void PIDSettingWidget::changeKp(double kp)
{
    assert(m_controller != 0);
    m_controller->setKp(kp);
}

void PIDSettingWidget::changeKd(double kd)
{
    assert(m_controller != 0);
    m_controller->setKd(kd);
}

void PIDSettingWidget::changeKi(double ki)
{
    assert(m_controller != 0);
    m_controller->setKi(ki);
}
