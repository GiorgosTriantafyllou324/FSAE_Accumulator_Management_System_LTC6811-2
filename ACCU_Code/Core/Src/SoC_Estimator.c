/*
 * SoC_Estimator.c
 *
 *  Created on: Jul 19, 2023
 *      Author: gamin
 */

#include "SoC_Estimator.h"

/*
 * TinyEKF: Extended Kalman Filter for embedded processors
 *
 * Copyright (C) 2015 Simon D. Levy
 *
 * MIT License
 */

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* Cholesky-decomposition matrix-inversion code, adapated from
   http://jean-pierre.moreau.pagesperso-orange.fr/Cplus/choles_cpp.txt */

static int choldc1(double * a, double * p, int n) {
    int i,j,k;
    double sum;

    for (i = 0; i < n; i++) {
        for (j = i; j < n; j++) {
            sum = a[i*n+j];
            for (k = i - 1; k >= 0; k--) {
                sum -= a[i*n+k] * a[j*n+k];
            }
            if (i == j) {
                if (sum <= 0) {
                    return 1; /* error */
                }
                p[i] = sqrt(sum);
            }
            else {
                a[j*n+i] = sum / p[i];
            }
        }
    }

    return 0; /* success */
}

static int choldcsl(double * A, double * a, double * p, int n)
{
    int i,j,k; double sum;
    for (i = 0; i < n; i++)
        for (j = 0; j < n; j++)
            a[i*n+j] = A[i*n+j];
    if (choldc1(a, p, n)) return 1;
    for (i = 0; i < n; i++) {
        a[i*n+i] = 1 / p[i];
        for (j = i + 1; j < n; j++) {
            sum = 0;
            for (k = i; k < j; k++) {
                sum -= a[j*n+k] * a[k*n+i];
            }
            a[j*n+i] = sum / p[j];
        }
    }

    return 0; /* success */
}

static int cholsl(double * A, double * a, double * p, int n)
{
    int i,j,k;
    if (choldcsl(A,a,p,n)) return 1;
    for (i = 0; i < n; i++) {
        for (j = i + 1; j < n; j++) {
            a[i*n+j] = 0.0;
        }
    }
    for (i = 0; i < n; i++) {
        a[i*n+i] *= a[i*n+i];
        for (k = i + 1; k < n; k++) {
            a[i*n+i] += a[k*n+i] * a[k*n+i];
        }
        for (j = i + 1; j < n; j++) {
            for (k = j; k < n; k++) {
                a[i*n+j] += a[k*n+i] * a[k*n+j];
            }
        }
    }
    for (i = 0; i < n; i++) {
        for (j = 0; j < i; j++) {
            a[i*n+j] = a[j*n+i];
        }
    }

    return 0; /* success */
}

static void zeros(double *a, int m, int n)
{
    for (uint8_t j=0; j<m*n; ++j)
        a[j] = 0;
}

/* C <- A * B */
static void mulmat(double * a, double * b, double * c, int arows, int acols, int bcols)
{
    for(uint8_t i=0; i<arows; ++i)
        for(uint8_t j=0; j<bcols; ++j)
        {
            c[i*bcols+j] = 0;
            for(uint8_t l=0; l<acols; ++l)
                c[i*bcols+j] += a[i*acols+l] * b[l*bcols+j];
        }
}

static void mulvec(double * a, double * x, double * y, int m, int n)
{
    int i, j;

    for(i=0; i<m; ++i) {
        y[i] = 0;
        for(j=0; j<n; ++j)
            y[i] += x[j] * a[i*n+j];
    }
}

static void transpose(double * a, double * at, int m, int n)
{
    for(uint8_t i=0; i<m; ++i)
    {
        for(uint8_t j=0; j<n; ++j)
            at[j*m+i] = a[i*n+j];
    }
}

/* A <- A + B */
static void accum(double * a, double * b, int m, int n)
{
    for(uint8_t i=0; i<m; ++i)
        for(uint8_t j=0; j<n; ++j)
            a[i*n+j] += b[i*n+j];
}

static void negate(double * a, int m, int n)
{
    for(uint8_t i=0; i<m; ++i)
        for(uint8_t j=0; j<n; ++j)
            a[i*n+j] = -a[i*n+j];
}

static void mat_addeye(double * a, int n)
{
	for (uint8_t i=0; i<n; ++i)
        a[i*n+i] += 1;
}

void ekf_init(ekf_t *ekf)
{
    /* Unpack rest of incoming structure
     * for initIalization - zero-out matrices */
    zeros(*(ekf->P), N, N);
    zeros(*(ekf->Q), N, N);
    zeros(*(ekf->R), M, M);
    zeros(*(ekf->G), N, M);
    zeros(*(ekf->F), N, N);
    zeros(*(ekf->H), M, N);

    /* Initialize noise covariance's */
    ekf->Q[0][0] = 7e-8;
    ekf->Q[1][1] = 3e-8;
    ekf->R[0][0] = 0.0017;

    /* Assume initial state at 100% SoC: It will be updated from the SoC_estimator
     * We don't assume degrading battery model: SoC[] isn't function of discharge-cycles */
    ekf->x[0] = 1;	// suppose close to 100% SoC
    ekf->x[1] = 0;	// 0V at RC branch
}

/* EKF step: at sample k for any given input matrix & Error_Covariance matrix */
uint8_t ekf_step(ekf_t *ekf, double *z)
{
    /* P_k = F_{k-1} P_{k-1} F^T_{k-1} + Q_{k-1} */
    mulmat(*(ekf->F), *(ekf->P), *(ekf->tmp0), N, N, N);
    transpose(*(ekf->F), *(ekf->Ft), N, N);
    mulmat(*(ekf->tmp0), *(ekf->Ft), *(ekf->Pp), N, N, N);
    accum(*(ekf->Pp), *(ekf->Q), N, N);

    /* G_k = P_k H^T_k (H_k P_k H^T_k + R)^{-1} */
    transpose(*(ekf->H), *(ekf->Ht), M, N);
    mulmat(*(ekf->Pp), *(ekf->Ht), *(ekf->tmp1), N, N, M);
    mulmat(*(ekf->H), *(ekf->Pp), *(ekf->tmp2), M, N, N);
    mulmat(*(ekf->tmp2), *(ekf->Ht), *(ekf->tmp3), M, N, M);
    accum(*(ekf->tmp3), *(ekf->R), M, M);
    if(cholsl(*(ekf->tmp3), *(ekf->tmp4), *(ekf->tmp5), M)) return 1;
    mulmat(*(ekf->tmp1), *(ekf->tmp4), *(ekf->G), N, M, M);

    /* \hat{x}_k = \hat{x_k} + G_k(z_k - h(\hat{x}_k)) */
    for(uint8_t i=0; i<M; ++i)
    	ekf->tmp7[i] = (z[i] - ekf->hx[i]);

    mulvec(*(ekf->G), ekf->tmp7, ekf->tmp6, N, M);

    for(uint8_t i=0; i<N; ++i)
        ekf->x[i] = (ekf->tmp6[i] + ekf->fx[i]);

    /* P_k = (I - G_k H_k) P_k */
    mulmat(*(ekf->G), *(ekf->H), *(ekf->tmp0), N, M, N);
    negate(*(ekf->tmp0), N, N);
    mat_addeye(*(ekf->tmp0), N);
    mulmat(*(ekf->tmp0), *(ekf->Pp), *(ekf->P), N, N, N);

    /* success */
    return 0;
}


/**
   Performs one step of the prediction and update.
 * @param z observation vector, length <i>m</i>
 * @return true on success, false on failure caused by non-positive-definite matrix.
 */
uint8_t SoC_update_step(ekf_t *ekf, SoC_estimator *soc, float current, float min_volt, double * z)
{
	/* Estimation values */
	soc->SoC_estimate 		   = ekf->x[0];
	soc->V1_estimate  		   = ekf->x[1];
	soc->cell_voltage_estimate = ekf->hx[0];

	/* Li-Ion MELASTA Battery Modeling, using experiment data:
	 * - Constant discharge 0.2C,10C
	 * - RC experiment at T25
	 * This function changes battery parameters based on the estimated change in SoC */
	update_accu_model(soc->SoC_estimate, &(soc->accu_param), min_volt);

	/* x[k+1] = f(x[k],I_current) + w[k] */
	ekf->fx[0] = ekf->x[0] - (soc->accu_param.Ts / 3600 * soc->accu_param.Cq) * current;
	ekf->fx[1] = ekf->x[1] * (1 - (soc->accu_param.Ts / (soc->accu_param.R1 * soc->accu_param.C1))) + ((soc->accu_param.Ts / soc->accu_param.C1) * current);

	/* y[k] = accu_voltage = h(x[k],I_current) + v[k] */
	ekf->hx[0] = soc->accu_param.Em - (soc->accu_param.R0 * current) - ekf->x[1];

	/* Jacobian_Matrix of f(.) */
	ekf->F[0][0] = 1;
	ekf->F[0][1] = 0;
	ekf->F[1][0] = 0;
	ekf->F[1][1] = 1 - (soc->accu_param.Ts / (soc->accu_param.R1 * soc->accu_param.C1));

	/* Jacobian_Matrix of h(.) */
	ekf->H[0][0] = 0;
	ekf->H[0][1] = -1;

	return ekf_step(ekf, z);
}


/* This function takes the pre-estimated value of SoC and produces min_cell(.) parameters */
void update_accu_model(double soc, accu_model *accu_param, float min_volt)
{
	accu_param->Ts = 0.5;	// 500ms sampling

	/* Must be calculated from RC experiments */
	accu_param->Em = min_volt;	// Must be drawn from LUT of RC experiments - MUST BE CHANGED
	accu_param->R0 = 0.0012;
	accu_param->R1 = 0.0002;
	accu_param->C1 = 1000;
	accu_param->Cq = 7.77;
}


/* Mapping Ah value of each cell at the start of the BMS functionality to produce total_Ah of the Accumulator
 * - Produces limits of Capacity_remaining based on Em-Ah curve
 * - With known voltage deviation from 3.1V, calculate based on the min_cell_voltage &
 * on the max_cell_voltage, the Ah remaining from P23 accumulator */
float find_ah_from_voltage(float voltage)
{
	float ah, voltage1, ah1, voltage2, ah2;
    for(uint8_t i = 0; i < sizeof(Em_Ah_LUT); i++)
    {
        voltage2 = Em_Ah_LUT[i][0];
        ah2	     = Em_Ah_LUT[i][1];

        voltage1 = Em_Ah_LUT[i + 1][0];
        ah1      = Em_Ah_LUT[i + 1][1];

        /* Calculate the corresponding Ah based on the linear interpolation */
        if((voltage1 <= voltage) &&  (voltage <= voltage2) && (voltage1 != voltage2))
        {
            ah = ah1 + (voltage - voltage1) * (ah2 - ah1) / (voltage2 - voltage1);
            return (2 * ah);
        }
    }

    return 0;
}


