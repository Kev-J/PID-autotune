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


#ifndef GRAPH_SETTINGS_WIDGET_H
#define GRAPH_SETTINGS_WIDGET_H

#include <QWidget>
#include <QDoubleSpinBox>

#include "../third_party/qcustomplot/qcustomplot.h"

class GraphSettingsWidget : public QWidget {
    Q_OBJECT;
    public:
        GraphSettingsWidget(QCustomPlot *plot, QWidget *parent = 0);

    private slots:
        void changeXMin(double xMin);
        void changeXMax(double xMax);
        void changeYMin(double yMin);
        void changeYMax(double yMax);

    private:
        bool m_mouseClicked;
        int m_mouseX, m_mouseY;
        QCustomPlot *m_plot;

        QDoubleSpinBox *m_xMinSpBx;
        QDoubleSpinBox *m_xMaxSpBx;
        QDoubleSpinBox *m_yMinSpBx;
        QDoubleSpinBox *m_yMaxSpBx;

};

#endif
