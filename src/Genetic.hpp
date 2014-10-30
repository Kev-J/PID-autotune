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

#ifndef GENETIC_H
#define GENETIC_H

#include <vector>

#include "Motor.hpp"
#include "PIDController.hpp"

class Genetic {
    public:
        Genetic(Motor *motor, double overshootPenalty = 0.0);

        void generatePopulation();
        void updatePopulation();
        PIDController getBestController(void);

        unsigned int getPopulationSize(void);
        double getCrossoverRatio(void);
        double getMutationRatio(void);
        double getEvalTime(void);
        double getOvershootPenalty(void) { return m_overshootPenalty; }
        double getInput(void) { return m_input; }
        int getEliteNum(void) { return m_eliteNum; }
        unsigned int getIterations(void) { return m_iterations; }

        void setPopulationSize(int size);
        void setCrossoverRatio(double crossoverRatio);
        void setMutationRatio(double mutationRatio);
        void setEvalTime(double seconds);
        void setMotor(Motor *motor);
        void setInput(double input) { m_input = input; }
        void setOvershootPenalty(double overshootPenalty) { m_overshootPenalty = overshootPenalty; }
        void setEliteNum(int eliteNum) { m_eliteNum = eliteNum > m_populationSize ? m_populationSize : eliteNum; }

        void setMinKp(double minKp);
        void setMinKd(double minKd);
        void setMinKi(double minKi);

        void setMaxKp(double maxKp);
        void setMaxKd(double maxKd);
        void setMaxKi(double maxKi);

    private:
        double randf(double min, double max);
        double grandf(void);
        double getFitness(PIDController controller);
        PIDController tournamentSelection();
        void arithmeticCrossover(PIDController &offspring1, PIDController &offspring2, PIDController parent1, PIDController parent2);
        void gaussianMutation(PIDController &offspring);

        Motor *m_motor;
        unsigned int m_iterations;
        double m_input;
        double m_populationSize;
        double m_evalTime;
        double m_overshootPenalty;
        std::vector<PIDController> m_PIDPopulation;
        std::vector<PIDController> m_parents;
        std::vector<double> m_fitness;
        double m_maxKp, m_maxKd, m_maxKi;
        double m_minKp, m_minKd, m_minKi;
        double m_crossoverRatio;
        double m_mutationRatio;
        double m_mutationScale;
        int m_eliteNum;

        PIDController m_bestController;
        double m_bestFitness;
};

#endif
