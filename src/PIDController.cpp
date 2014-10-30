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

#include "PIDController.hpp"

PIDController::PIDController()
:m_kp(0.0), m_kd(0.0), m_ki(0.0), m_inOld(0), m_sum(0)
{
    m_name = "PID";
}

double PIDController::sample(double in)
{
    double output;

    output = m_kp*in;
    output += m_kd*(in - m_inOld);
    output += m_ki*m_sum;

    m_inOld = in;
    m_sum += in;

    return output;
}

void PIDController::reset(void)
{
    m_inOld = 0;
    m_sum = 0;
}

double PIDController::getKp(void)
{
    return m_kp;
}

double PIDController::getKd(void)
{
    return m_kd;
}

double PIDController::getKi(void)
{
    return m_ki;
}

void PIDController::setKp(double kp)
{
    m_kp = kp;
}

void PIDController::setKd(double kd)
{
    m_kd = kd;
}

void PIDController::setKi(double ki)
{
    m_ki = ki;
}
