#include "linalg.h"

#include <math.h>

bool linalg_solve(double *A, double *b, int n, double *x) {
    for (int col = 0; col < n; col++) {
        int piv = col;
        double best = fabs(A[col * n + col]);
        for (int r = col + 1; r < n; r++) {
            double v = fabs(A[r * n + col]);
            if (v > best) { best = v; piv = r; }
        }
        if (best < 1e-14) return false;

        if (piv != col) {
            for (int k = 0; k < n; k++) {
                double t = A[col * n + k];
                A[col * n + k] = A[piv * n + k];
                A[piv * n + k] = t;
            }
            double t = b[col];
            b[col] = b[piv];
            b[piv] = t;
        }

        double diag = A[col * n + col];
        for (int r = col + 1; r < n; r++) {
            double factor = A[r * n + col] / diag;
            if (factor == 0.0) continue;
            for (int k = col; k < n; k++) {
                A[r * n + k] -= factor * A[col * n + k];
            }
            b[r] -= factor * b[col];
        }
    }

    for (int i = n - 1; i >= 0; i--) {
        double s = b[i];
        for (int k = i + 1; k < n; k++) {
            s -= A[i * n + k] * x[k];
        }
        x[i] = s / A[i * n + i];
    }
    return true;
}
