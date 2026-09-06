#ifndef LINALG_H
#define LINALG_H

#include <stdbool.h>

/* Solves the n x n system A x = b via Gaussian elimination with partial
 * pivoting. A (row-major, n*n) and b (length n) are used as scratch space
 * and modified in place; the solution is written to x (length n). Returns
 * false if the system is (numerically) singular. */
bool linalg_solve(double *A, double *b, int n, double *x);

#endif
