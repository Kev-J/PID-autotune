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

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include "Controller.hpp"

class PIDController : public Controller {
    public:
        PIDController();
        double sample(double in);
        void reset(void);
        double getKp(void);
        double getKd(void);
        double getKi(void);
        void setKp(double kp);
        void setKd(double kd);
        void setKi(double ki);

    private:
        double m_kp, m_ki, m_kd;
        double m_sum;
        double m_inOld;
};

#endif
