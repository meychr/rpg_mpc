# ACADO with MLP
ACADO implementation of a Multi-Layer Perceptron. Basic examples for starters, namely, how to define an MLP structure to be used in functions/ODE.

## Warning
If your network is too complicated/large with many nodes, it can be that the code generation for ACADO does not work
because the resulting code would be too large for your RAM. In this case, try to start with a smaller network.

## Code Generation

For code generation have a look at the general README in this folder (source ACADO, cmake and make, then execute code).

## Basic usage

Load your net parameters (i.e., txt files containing the weights and bias for each layer). They should be in the same folder.

    //Load the net parameters (weights, bias).
    
    DMatrix W1; W1.read( "W1.txt" );
    DMatrix W2; W2.read( "W2.txt" );
    DMatrix B1; B1.read( "B1.txt" );
    DMatrix B2; B2.read( "B2.txt" );
    
  Define the elements of your problem

    DifferentialState x("", 2, 1);
    IntermediateState features(2); //here you will set your net input 
    IntermediateState layer1, x1, layer2;   
    Function f;
 
Define your features vector and write the expression of your MLP.
 
    // Feature vector

    features(0)=x(0);
    features(1)=x(1);

    //Write your MLP expression
    layer1 = W1*x + B1;
    x1 =  tanh(layer1);
    layer2 = W2*x1+B2;
    ...
    
Finally, use your MLP in your function

    f << layer2;