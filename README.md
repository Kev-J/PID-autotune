PID-autotune
============

A self-tuning PID demonstration GPL software using genetic algorithm.

Demonstration video here : https://www.youtube.com/watch?v=cK6kWN9K_do

Explanation here : https://kevinjoly25.wordpress.com/2015/01/13/pid-controller-auto-tuning-using-genetic-algorithm/

Dependencies
============

- Qt4

Building
============

$ mkdir build  
$ cd build  
$ cmake ..  
$ make  

Installing
============

No install method has been provided yet. However, you can run the software from the build directory:
$ ./pid-autotune

Usage
============

There is 4 dock widgets in this software:
 - Motor: enable the user to choose a motor to use and test it in closed on opened loop.
 - Controller : enable the user to choose a controller to use with the motor (check "Use controller"). The controller parameters can be set in this widget for test purpose.
 - Graph settings : enable the user to change the axes scale by setting the min and max to be displayed.
 - Genetic : enable the user to control the genetic algorithm parameters such as:
 	* input : value of the input applied on the system.
	* min/max Kx : boundary values of each PID action.
	* Evaluation time : system running time when evaluating fitness.
	* Population size : size of the genetic algorithm's population.
	* Mutation ratio : probability to mutate the offspring's variable.
	* Crossover ratio : probability to crossover two parents.
	* Overshoot penalty : ratio which multiply the error when an overshoot occurs. If you don't want any overshoot, set this to the maximum.
	* Elite num : Number of best parents kept in the next generation of population.
	The start button launch the genetic process. Pause stop the process, press start to launch it again without any loss. Reset enable the user to generate a new random population by deleting the old one.

Example
------------

- Under "Motor" : choose the DummyMotor.
- Under "Graph settings" : set xMax to 0.1 and yMax to 2.0.
- Under "Genetic" : set maxKp to 1.0, maxKd to 2.0, maxKi to 0.1.
- Hit start button and enjoy the dance of a self-tuning PID! ;)

More on GAs...
------------

The fitness function is using the sum of squarred error to evaluate the generated PID.
Thanks to this fitness function, tournament selection can be used in order to select parents of the next PID population.
The genetic algorithm implemented in Genetic.cpp uses arithmetic crossover and gaussian mutation to generate the new population.
Elitism can be used.

Third party
============

This software is using the GPL software QCustomPlot from Emanuel Eichhammer.
