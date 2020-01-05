# acado_mlp_examples
ACADO implementation of a Multi-Layer Perceptron. Basic examples for starters, namely, how to define an MLP structure to be used in functions/ODE.

## Starting

For instant test, just clone this into the ~/ACADOtoolkit/examples/ directory. 

    cd ~/ACADOtoolkit/build
    cmake ..
    make

Then

    cd ~/ACADOtoolkit/examples/acado_mlp_examples
    ./mlp_function
or any other file you write. Any time you make changes, you have to cmake and make as above. There's 100% a smarter way but for the time being, it be like that.

## Basic usage

Making reference to `mlp_function.cpp` 
(**NOTE:** THIS IS NOT OCP! Just a function evaluation, but you ideally can copy this code in an OCP problem to use in ODEs etc.):

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

Then, do your evaluations.

Now, making reference to `mlp_function_ode.cpp` : this was just a test to see if everything explodes. In this particular case, it doesn't, but it is not significative imho.

## To do

 1. Check for the whole code generation thingy
 2. Train an MLP of an toy process (input: u, output: states)
 3. Use said MLP instead of explicit equations in the ODE definition
 4. Check that results are comparable with explicit definition
 5. ???
 6. Profit.

