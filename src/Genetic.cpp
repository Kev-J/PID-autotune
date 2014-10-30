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

#include "Genetic.hpp"

#include <iostream>
#include <limits>
#include <assert.h>
#include <math.h>

#define MIN(a,b) (a < b ? a : b)
#define MAX(a,b) (a > b ? a : b)

Genetic::Genetic(Motor *motor, double overshootPenalty)
:m_motor(motor), m_overshootPenalty(overshootPenalty),
    m_iterations(0), m_populationSize(0), m_input(1.0),
    m_minKp(0.0), m_minKd(0.0), m_minKi(0.0),
    m_maxKp(0.0), m_maxKd(0.0), m_maxKi(0.0),
    m_bestFitness(0.0), m_crossoverRatio(0.75) , m_mutationRatio(0.01), m_mutationScale(100), m_evalTime(1.0),
    m_eliteNum(0)
{
    m_bestController.setKp(0.0);
    m_bestController.setKd(0.0);
    m_bestController.setKi(0.0);
}

void Genetic::setMotor(Motor *motor)
{
    m_motor = motor;
}

void Genetic::setMinKp(double minKp)
{
    m_minKp = minKp;
    generatePopulation();
}

void Genetic::setMinKd(double minKd)
{
    m_minKd = minKd;
    generatePopulation();
}

void Genetic::setMinKi(double minKi)
{
    m_minKi = minKi;
    generatePopulation();
}

void Genetic::setMaxKp(double maxKp)
{
    m_maxKp = maxKp;
    generatePopulation();
}

void Genetic::setMaxKd(double maxKd)
{
    m_maxKd = maxKd;
    generatePopulation();
}

void Genetic::setMaxKi(double maxKi)
{
    m_maxKi = maxKi;
    generatePopulation();
}

void Genetic::setPopulationSize(int size)
{
    assert(size>0);
    assert((size%2) == 0);

    m_fitness.resize(size);
    m_populationSize = size;
    generatePopulation();

    m_eliteNum = MIN(m_eliteNum, m_populationSize);
}

void Genetic::setCrossoverRatio(double crossoverRatio)
{
    m_crossoverRatio = MAX(MIN(crossoverRatio, 1.0), 0.0);
}

void Genetic::setMutationRatio(double mutationRatio)
{
    m_mutationRatio = MAX(MIN(mutationRatio, 1.0), 0.0);
}

void Genetic::setEvalTime(double seconds)
{
    m_evalTime = seconds;
}

void Genetic::generatePopulation()
{
    m_bestFitness = 0.0;

    m_iterations = 0;
    m_PIDPopulation.clear();
    for (unsigned int i = 0 ; i < m_populationSize ; i++) {
        PIDController controller;
        controller.setKp(randf(m_minKp, m_maxKp));
        controller.setKd(randf(m_minKd, m_maxKd));
        controller.setKi(randf(m_minKi, m_maxKi));
        m_PIDPopulation.push_back(controller);
    }
}

double Genetic::getFitness(PIDController controller)
{
    double samplingPer = m_motor->getSamplingPeriod();
    int nbSamples = (int)(m_evalTime / samplingPer);

    assert(m_overshootPenalty >= 0.0);

    m_motor->setController(&controller);

    QVector<double> y(nbSamples);
    m_motor->runClosedLoop(y, m_input, m_evalTime);

    double error2 = 0.0;
    for (int i = 0 ; i < nbSamples ; i++) {
        if (y[i] > m_input) // overshoot -> apply penalty
            error2 += (m_input-y[i])*(m_input-y[i])*(1.0+m_overshootPenalty);
        else
            error2 += (m_input-y[i])*(m_input-y[i]);
    }

    if (error2 == 0.0)
        return std::numeric_limits<double>::max();
    else
        return 1.0/error2;
}

// TODO be able to set challenger number
PIDController Genetic::tournamentSelection()
{
    assert(m_fitness.size() == m_parents.size());

    unsigned int parent1Idx = rand() % m_parents.size();
    unsigned int parent2Idx = rand() % (m_parents.size()-1);

    if (parent2Idx >= parent1Idx)
        parent2Idx++;

    if (m_fitness[parent1Idx] > m_fitness[parent2Idx]) {
        return m_parents[parent1Idx];
    } else {
        return m_parents[parent2Idx];
    }
}

void Genetic::arithmeticCrossover(PIDController &offspring1, PIDController &offspring2, PIDController parent1, PIDController parent2)
{
    if (randf(0.0, 1.0) < m_crossoverRatio) {
        // Do crossover
        double weightKp = randf(0.0, 1.0);
        double weightKd = randf(0.0, 1.0);
        double weightKi = randf(0.0, 1.0);

        // P action
        double kp1 = parent1.getKp()*weightKp + parent2.getKp()*(1.0-weightKp);
        double kp2 = parent1.getKp()*(1.0-weightKp) + parent2.getKp()*weightKp;

        MIN(MAX(kp1, m_minKp), m_maxKp);
        MIN(MAX(kp2, m_minKp), m_maxKp);

        offspring1.setKp(kp1);
        offspring2.setKp(kp2);

        // D action
        double kd1 = parent1.getKd()*weightKd + parent2.getKd()*(1.0-weightKd);
        double kd2 = parent1.getKd()*(1.0-weightKd) + parent2.getKd()*weightKd;

        MIN(MAX(kd1, m_minKd), m_maxKd);
        MIN(MAX(kd2, m_minKd), m_maxKd);

        offspring1.setKd(kd1);
        offspring2.setKd(kd2);

        // I action
        double ki1 = parent1.getKi()*weightKi + parent2.getKi()*(1.0-weightKi);
        double ki2 = parent1.getKi()*(1.0-weightKi) + parent2.getKi()*weightKi;

        MIN(MAX(ki1, m_minKi), m_maxKi);
        MIN(MAX(ki2, m_minKi), m_maxKi);

        offspring1.setKi(ki1);
        offspring2.setKi(ki2);
    } else { // Parents survives
        offspring1 = parent1;
        offspring2 = parent2;
    }
}

void Genetic::gaussianMutation(PIDController &offspring)
{
    //TODO do not hardcode 10.0
    // TODO try to use two mutation operator with different sigma (change 10.0 by 100.0 or 1.0).
    // Idea according to this: http://www.iue.tuwien.ac.at/phd/heitzinger/node27.html
    if (randf(0.0, 1.0) < m_mutationRatio) {
        double sigmaKp = (m_maxKp-m_minKp)/10.0;
        double kp = offspring.getKp();
        kp += grandf()*sigmaKp;

        // Clip values
        kp = MAX(MIN(kp,m_maxKp), m_minKp);

        offspring.setKp(kp);
    }

    if (randf(0.0, 1.0) < m_mutationRatio) {
        double sigmaKd = (m_maxKd-m_minKd)/10.0;
        double kd = offspring.getKd();
        kd += grandf()*sigmaKd;

        // Clip values
        kd = MAX(MIN(kd,m_maxKd), m_minKd);

        offspring.setKd(kd);
    }

    if (randf(0.0, 1.0) < m_mutationRatio) {
        double sigmaKi = (m_maxKi-m_minKi)/10.0;
        double ki = offspring.getKi();
        ki += grandf()*sigmaKi;

        // clip values
        ki = MAX(MIN(ki,m_maxKi), m_minKi);

        offspring.setKi(ki);
    }

}

void Genetic::updatePopulation()
{

    assert(m_PIDPopulation.size() > 0);
    assert(m_PIDPopulation.size() == m_populationSize);

    m_parents.clear();

    // Get fitness
    for (unsigned int i = 0 ; i < m_PIDPopulation.size() ; i++) {
        m_fitness[i] = getFitness(m_PIDPopulation[i]);
        m_parents.push_back(m_PIDPopulation[i]);
        if (m_bestFitness < m_fitness[i]) {
            m_bestFitness = m_fitness[i];
            m_bestController = m_PIDPopulation[i];
        }
    }

    // Sort
    for (unsigned int i = 0 ; i < m_parents.size() ; i++) {
        unsigned int bestIdx = i;
        double bestFitness = m_fitness[i];
        for (unsigned int j = i ; j < m_parents.size() ; j++) {
            if (m_fitness[j] > bestFitness) {
                bestIdx = j;
                bestFitness = m_fitness[j];
            }
        }

        PIDController tmpController = m_parents[i];
        double tmpFitness = m_fitness[i];

        m_parents[i] = m_parents[bestIdx];
        m_fitness[i] = m_fitness[bestIdx];

        m_parents[bestIdx] = tmpController;
        m_fitness[bestIdx] = tmpFitness;
    }

    m_PIDPopulation.clear();

    if (m_eliteNum) {
        for (unsigned int i = 0 ; i < m_eliteNum ; i++) {
            m_PIDPopulation.push_back(m_parents[i]);
        }
    }

    while (m_PIDPopulation.size() < m_populationSize) {
        PIDController offspring1, offspring2;

        // TODO implement roulette-wheel selection as well
        // Tournament selection
        PIDController parent1 = tournamentSelection();
        PIDController parent2 = tournamentSelection();

        // Crossover
        arithmeticCrossover(offspring1, offspring2, parent1, parent2);

        // Mutation
        gaussianMutation(offspring1);
        gaussianMutation(offspring2);

        m_PIDPopulation.push_back(offspring1);
        m_PIDPopulation.push_back(offspring2);
    }
    m_iterations++;
}

double Genetic::randf(double min, double max)
{
    double d = (double)rand() / (double)RAND_MAX;
    return min + d * (max - min);
}

// Box-Muller method
double Genetic::grandf()
{
    static bool generated = false;
    double x1;
    static double x2,w;


    if (generated) {
        generated = false;
        return x2*w;
    }

    do {
        x1 = 2.0 * randf(0.0, 1.0) - 1.0;
        x2 = 2.0 * randf(0.0, 1.0) - 1.0;
        w = x1*x1 + x2*x2;
    } while (w >= 1.0);

    w = sqrt((-2.0*log(w))/w);

    generated = true;

    return x1*w;
}

PIDController Genetic::getBestController(void)
{
    return m_bestController;
}

unsigned int Genetic::getPopulationSize(void)
{
    return m_populationSize;
}

double Genetic::getCrossoverRatio(void)
{
    return m_crossoverRatio;
}

double Genetic::getMutationRatio(void)
{
    return m_mutationRatio;
}

double Genetic::getEvalTime(void)
{
    return m_evalTime;
}
