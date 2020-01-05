/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2014 by Boris Houska, Hans Joachim Ferreau,
 *    Milan Vukov, Rien Quirynen, KU Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC)
 *    under supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



 /**
 *    \file examples/matrix_vector/matrix_from_file.cpp
 *    \author Boris Houska, Hans Joachim Ferreau
 *    \date 2008
 */


#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/user_interaction/user_interaction.hpp>
#include <acado/symbolic_expression/symbolic_expression.hpp>
#include <acado/function/function.hpp>

using namespace std;

USING_NAMESPACE_ACADO

IntermediateState dotDivide( IntermediateState x , IntermediateState y ){
    /*  Implementation of elemen-wise division of two vectors   */

    IntermediateState dotDiv(x.getDim());
    int i;
    for(i=0; i<x.getDim(); i++){
        dotDiv(i)=x(i) / y(i);
    }
    return dotDiv; 

}

IntermediateState tanh( IntermediateState x ){
    /*  This is the hyperbolic tangent function. Most activation functions are not 
        implemented in ACADO, so you must define them as INTERMEDIATE STATES.   */

    DMatrix I( x.getNumRows(), x.getNumCols());
    I.setAll(1);    //this is apparently the only way to sum a constant value element-wise.
    IntermediateState numTanh=(I-exp(-2*x));
    IntermediateState denTanh=(I+exp(-2*x));

    return dotDivide(numTanh,denTanh); 

}


/* >>> start tutorial code >>> */
int main( )
{
    
    //Load the net parameters (weights, bias).
	DMatrix W1; W1.read( "W1.txt" );
    DMatrix W2; W2.read( "W2.txt" );
    DMatrix B1; B1.read( "B1.txt" );
    DMatrix B2; B2.read( "B2.txt" );

    // W1.print();
    // B1.print();
    // W2.print();
    // B2.print();



    DifferentialState      x("", 2, 1);
    IntermediateState      features(2); //here you will set your net input
    IntermediateState      layer1, x1, layer2;
    Function               f;

    // Feature vector

    features(0)=x(0);
    features(1)=x(1);


    //Write your MLP expression
    layer1 = W1*x + B1;
    x1 = tanh(layer1);
    layer2 = W2*x1+B2;

    printf("layer1   %d\n", layer1.getDim());  
    printf("layer12   %d\n", x1.getDim());
    printf("layer2     %d\n", layer2.getDim());

    f << layer2;
    
    f.print(std::cout, "func");

    printf("the dimension of f is      %d                      \n",  f.getDim() );
    printf("the function f depends on  %d  differential states \n",  f.getNX () );
    printf("the function f depends on  %d  controls            \n",  f.getNU () );
    printf("the function f depends on  %d  parameters          \n",  f.getNP () );

    // TEST THE FUNCTION f:
    // --------------------
       EvaluationPoint zz(f);

       DVector xx(2);
       xx(0) = 2.0;
       xx(1) = 3.0;

       zz.setX( xx );
       DVector result = f.evaluate( zz );

       result.print();

    return 0;
}
/* <<< end tutorial code <<< */