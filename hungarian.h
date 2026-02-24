#ifndef HUNGARIAN_H
#define HUNGARIAN_H

#define INF 1e9
#define MAX_N 10

#include <algorithm>

void hungarian(float cost[MAX_N][MAX_N], int n, int assignment[MAX_N]) {
  float u[MAX_N], v[MAX_N];
  int p[MAX_N], way[MAX_N];

  for (int i = 0; i <= n; ++i) {
    u[i] = 0;
    v[i] = 0;
    p[i] = 0;
  }

  for (int i = 1; i <= n; ++i) {
    p[0] = i;
    int j0 = 0;
    float minv[MAX_N];
    bool used[MAX_N] = {false};

    for (int j = 0; j <= n; ++j)
      minv[j] = INF;

    do {
      used[j0] = true;
      int i0 = p[j0], j1;
      float delta = INF;

      for (int j = 1; j <= n; ++j) {
        if (!used[j]) {
          float cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
      }

      for (int j = 0; j <= n; ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);

    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0);
  }

  for (int j = 1; j <= n; ++j)
    assignment[p[j] - 1] = j - 1;
}

#endif
