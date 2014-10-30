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

#ifndef ODD_SPIN_BOX_H
#define ODD_SPIN_BOX_H

#include <QSpinBox>

// Thanks to ChrisW67 from the Qtproject forum: 
// http://qt-project.org/forums/viewthread/24037
class OddSpinBox : public QSpinBox {
    Q_OBJECT;
    public:
    OddSpinBox(QWidget *parent = 0)
        : QSpinBox(parent)
    {
        setSingleStep(2);
    }

    protected:
    int valueFromText(const QString &text) const
    {
        int textIntLength = text.length() - prefix().length() - suffix().length();
        QString textInt = text.mid(prefix().length(), textIntLength);
        int value = textInt.toInt();
        if (value % 2 != 0)
            value--;

        value = qMax(value, minimum());
        value = qMin(value, maximum());

        return value;
    }
};

#endif
