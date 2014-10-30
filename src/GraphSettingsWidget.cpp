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

#include "GraphSettingsWidget.hpp"

#include <QGridLayout>
#include <QLabel>
#include <iostream>

GraphSettingsWidget::GraphSettingsWidget(QCustomPlot *plot, QWidget *parent)
:QWidget(parent), m_plot(plot)
{
    QGridLayout *layout = new QGridLayout;

    m_xMinSpBx = new QDoubleSpinBox;
    m_xMaxSpBx = new QDoubleSpinBox;
    m_yMinSpBx = new QDoubleSpinBox;
    m_yMaxSpBx = new QDoubleSpinBox;

    m_xMaxSpBx->setValue(1.0);
    m_yMaxSpBx->setValue(1.0);

    layout->addWidget(new QLabel("X min"), 0, 0);
    layout->addWidget(new QLabel("X max"), 0, 2);
    layout->addWidget(new QLabel("Y min"), 1, 0);
    layout->addWidget(new QLabel("Y max"), 1, 2);

    layout->addWidget(m_xMinSpBx, 0, 1);
    layout->addWidget(m_xMaxSpBx, 0, 3);
    layout->addWidget(m_yMinSpBx, 1, 1);
    layout->addWidget(m_yMaxSpBx, 1, 3);

    setLayout(layout);

    m_plot->xAxis->setRange(m_xMinSpBx->value(), m_xMaxSpBx->value());
    m_plot->yAxis->setRange(m_yMinSpBx->value(), m_yMaxSpBx->value());
    m_plot->replot();

    connect(m_xMinSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeXMin(double)));
    connect(m_xMaxSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeXMax(double)));
    connect(m_yMinSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeYMin(double)));
    connect(m_yMaxSpBx, SIGNAL(valueChanged(double)), this, SLOT(changeYMax(double)));

    //TODO add automatic scale thanks to : m_plot->graph(0)->rescaleAxes(false);
}

void GraphSettingsWidget::changeXMin(double xMin)
{
    if (xMin > m_xMaxSpBx->value()) {
        xMin = m_xMaxSpBx->value();
        m_xMinSpBx->setValue(xMin);
    }

    m_plot->xAxis->setRangeLower(xMin);
    m_plot->replot();
}

void GraphSettingsWidget::changeXMax(double xMax)
{
    if (xMax < m_xMinSpBx->value()) {
        xMax = m_xMinSpBx->value();
        m_xMaxSpBx->setValue(xMax);
    }

    m_plot->xAxis->setRangeUpper(xMax);
    m_plot->replot();
}

void GraphSettingsWidget::changeYMin(double yMin)
{
    if (yMin > m_yMaxSpBx->value()) {
        yMin = m_yMaxSpBx->value();
        m_yMinSpBx->setValue(yMin);
    }

    m_plot->yAxis->setRangeLower(yMin);
    m_plot->replot();
}

void GraphSettingsWidget::changeYMax(double yMax)
{
    if (yMax < m_yMinSpBx->value()) {
        yMax = m_yMinSpBx->value();
        m_yMaxSpBx->setValue(yMax);
    }

    m_plot->yAxis->setRangeUpper(yMax);
    m_plot->replot();
}
