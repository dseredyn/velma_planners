// Copyright (c) 2015, Robot Control and Pattern Recognition Group,
// Institute of Control and Computation Engineering
// Warsaw University of Technology
//
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Warsaw University of Technology nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Author: Dawid Seredynski
//

#include "velma_dyn_model.h"

#include <stdlib.h>
#include <math.h>

#include "params.inc"

DynModelVelma::DynModelVelma() {
    M_.resize(5,5);
    invM_.resize(5,5);
    tmpTau_.resize(5);
}

DynModelVelma::~DynModelVelma() {
}
 
void DynModelVelma::inertia(Eigen::MatrixXd &x, const Eigen::VectorXd &qq){

    Eigen::VectorXd q(16);
    q(0) = qq(0);
    q(1) = -3.141592653589793/2.0;  // torso_1_joint is fixed
    for (int q_idx = 2; q_idx < 16; q_idx++) {
        q(q_idx) = qq(q_idx-1);
    }

    x.setZero();
#include "inertia.inc"
}

void DynModelVelma::gaussjordan(const Eigen::MatrixXd &inMatrix, Eigen::MatrixXd &outMatrix, int dim){
 
	int iRow, iCol, diagIndex;
	double diagFactor, tmpFactor;
    Eigen::MatrixXd inMatrixCopy(inMatrix);
 
	/* make deep copy of input matrix */
	for(iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy(iCol,iRow) = inMatrix(iCol,iRow);
		}
	}
	/* Initialize output matrix as identity matrix. */
	for (iRow = 0; iRow < dim; iRow++ ){
		for (iCol = 0; iCol < dim; iCol++ ){
			if (iCol == iRow){
				outMatrix(iCol,iRow) = 1;
			}
			else{
				outMatrix(iCol,iRow) = 0;
			}
		}
	}
 
	for (diagIndex = 0; diagIndex < dim; diagIndex++ )
	{
		/* determine diagonal factor */
		diagFactor = inMatrixCopy(diagIndex,diagIndex);
 
		/* divide column entries by diagonal factor */
		for (iCol = 0; iCol < dim; iCol++){
			inMatrixCopy(iCol,diagIndex) /= diagFactor;
			outMatrix(iCol,diagIndex) /= diagFactor;
		}
 
		/* perform line-by-line elimination */
		for (iRow = 0; iRow < dim; iRow++){
			if (iRow != diagIndex){
				tmpFactor = inMatrixCopy(diagIndex,iRow);
 
				for(iCol = 0; iCol < dim; iCol++){
				inMatrixCopy(iCol,iRow)  -= inMatrixCopy(iCol,diagIndex)*tmpFactor;
				outMatrix(iCol,iRow) -= outMatrix(iCol,diagIndex)*tmpFactor;
				}
			}
		} /* line-by-line elimination */
 
	}
}

void DynModelVelma::computeM(const Eigen::VectorXd &q) {
	inertia(M_, q);
	gaussjordan(M_, invM_, 5);
}

void DynModelVelma::accel(Eigen::VectorXd &QDD, const Eigen::VectorXd &q, const Eigen::VectorXd &dq, const Eigen::VectorXd &t){

	/* declare variables */
	int iCol;

	/* call the computational routines */
//	coriolis(C_, q, dq);
//	gravload(gravload, q);
//	friction(friction, dq);
 
	/* fill temporary vector */
//    tmpTau_ = C_ * dq;
	for (iCol = 0; iCol < 5; iCol++){
//		tmpTau_[iCol] = t[iCol] -  tmpTau_[iCol] - gravload[iCol][0] + friction[iCol][0];
		tmpTau_[iCol] = t[iCol] - dq[iCol] * 0.1;
	}
	/* compute acceleration */
    QDD = invM_ * tmpTau_;
}

