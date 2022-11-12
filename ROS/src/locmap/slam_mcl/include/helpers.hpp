#ifndef _HELPERS_H_
#define _HELPERS_h_

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace slam
{

  // http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
  MatrixXf randn(int m, int n)
  {
    // use formula 30.3 of Statistical Distributions (3rd ed)
    // Merran Evans, Nicholas Hastings, and Brian Peacock
    int urows = m * n + 1;
    VectorXf u(urows);

    // u is a random matrix
#if 1
    for (int r = 0; r < urows; r++)
    {
      // FIXME: better way?
      u(r) = std::rand() * 1.0 / RAND_MAX;
    }
#else
    u = ((VectorXf::Random(urows).array() + 1.0) / 2.0).matrix();
#endif

    MatrixXf x(m, n);

    int i, j, k;
    float square, amp, angle;

    k = 0;
    for (i = 0; i < m; i++)
    {
      for (j = 0; j < n; j++)
      {
        if (k % 2 == 0)
        {
          square = -2. * std::log(u(k));
          if (square < 0.)
            square = 0.;
          amp = sqrt(square);
          angle = 2. * M_PI * u(k + 1);
          x(i, j) = amp * std::sin(angle);
        }
        else
          x(i, j) = amp * std::cos(angle);

        k++;
      }
    }

    return x;
  }

  MatrixXf rand(int m, int n)
  {
    MatrixXf x(m, n);
    int i, j;
    float rand_max = float(RAND_MAX);

    for (i = 0; i < m; i++)
    {
      for (j = 0; j < n; j++)
        x(i, j) = float(std::rand()) / rand_max;
    }
    return x;
  }

  float calculate_covariance(vector<float> &X, vector<float> &Y)
  {

    if (X.size() != Y.size())
    {
      throw invalid_argument("Size of X must equal size of Y");
    }

    unsigned int N = X.size();

    float meanX = reduce(X.begin(), X.end()) / static_cast<float>(N);
    float meanY = reduce(Y.begin(), Y.end()) / static_cast<float>(N);

    float covariance = 0.0;

    for (unsigned int i = 0; i < N; i++)
    {
      covariance += (X[i] - meanX) * (Y[i] - meanY);
    }

    return covariance / static_cast<float>(N);
  }

  VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n)
  {
    int len = x.size();

    // choleksy decomposition
    MatrixXf S = P.llt().matrixL();
    MatrixXf X = randn(len, n);

    return S * X + x;
  }

  void pi_to_pi(VectorXf &angle)
  {
    int i, n;

    for (i = 0; i < angle.size(); i++)
    {
      if ((angle[i] < (-2 * M_PI)) || (angle[i] > (2 * M_PI)))
      {
        n = floor(angle[i] / (2 * M_PI));
        angle[i] = angle[i] - n * (2 * M_PI);
      }

      if (angle[i] > M_PI)
        angle[i] = angle[i] - (2 * M_PI);

      if (angle[i] < -M_PI)
        angle[i] = angle[i] + (2 * M_PI);
    }
  }

  float pi_to_pi(float ang)
  {
    int n;

    if ((ang < -2 * M_PI) || (ang > 2 * M_PI))
    {
      n = floor(ang / (2 * M_PI));
      ang = ang - n * (2 * M_PI);
    }

    if (ang > M_PI)
      ang = ang - (2 * M_PI);

    if (ang < -M_PI)
      ang = ang + (2 * M_PI);

    return ang;
  }

  float pi_to_pi2(float ang)
  {
    if (ang > M_PI)
      ang = ang - (2 * M_PI);

    if (ang < -M_PI)
      ang = ang + (2 * M_PI);

    return ang;
  }

  double unifRand()
  {
    return std::rand() / double(RAND_MAX);
  }

  void stratified_random(unsigned long N, vector<float> &di)
  {
    float k = 1.0 / (float)N;

    // deterministic intervals
    float temp = k / 2;
    while (temp < (1 - k / 2))
    {
      di.push_back(temp);
      temp = temp + k;
    }

    // FIXME: when set NPARTICLES = 30, this whill failed
    assert(di.size() == N);

    // dither within interval
    vector<float>::iterator diter;
    for (diter = di.begin(); diter != di.end(); diter++)
    {
      *diter = (*diter) + unifRand() * k - (k / 2);
    }
  }

  void cumsum(VectorXf &w)
  {
    VectorXf csumVec(w.size());
    for (int i = 0; i < w.size(); i++)
    {
      float sum = 0;
      for (int j = 0; j <= i; j++)
      {
        sum += w(j);
      }
      csumVec(i) = sum;
    }

    w = VectorXf(csumVec); // copy constructor. Double check
  }

  void stratified_resample(VectorXf w, vector<int> &keep, float &Neff)
  {
    VectorXf wsqrd(w.size());
    double w_sum = w.sum();

    for (int i = 0; i < w.size(); i++)
    {
      // FIXME: matlab version is: w = w / sum(w)
      w(i) = w(i) / w_sum;
      wsqrd(i) = pow(w(i), 2);
    }
    Neff = 1.0 / (float)wsqrd.sum();

    int len = w.size();
    keep.resize(len);
    for (int i = 0; i < len; i++)
    {
      keep[i] = i;
    }

    vector<float> select;
    stratified_random(len, select);
    cumsum(w);

    int ctr = 0;
    for (int i = 0; i < len; i++)
    {
      while ((ctr < len) && (select[ctr] < w(i)))
      {
        keep[ctr] = i;
        ctr++;
      }
    }
  }
}

#endif