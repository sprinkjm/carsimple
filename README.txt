Copyright (c) 2021
Vanderbilt University Board of Regents
Portions Copyright (c) 2019 Arizona Board of Regents
Author: Jonathan Sprinkle, PhD
        With contributions from others throughout the years

Use this repository to simulate the output of a car-like robot. 

The repository includes several models:
(1) a more complex kinematic model with steering rate inputs
(2) a simple kinematic model similar to Dubins

model1.m and model2.m provide input/output as matlab functions.

simulink_exexecution provides a graphical example with two different inputs.

HOW TO USE

1. Open the simulink_execution model, and execute it.
2. Run one of the matlab versions

    simulateModel1('input5.txt','output5_1.txt')
    simulateModel2('input5.txt','output5_2.txt')
and then visualize the difference
    animateCar('output5_1.txt','output5_2.txt')

3. Generate your own inputs, using the format defined in generateInputs.m

