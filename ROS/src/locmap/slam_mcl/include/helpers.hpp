#ifndef _HELPERS_H_
#define _HELPERS_h_

#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

namespace slam
{

  double unifRand()
  {
    return rand() / double(RAND_MAX);
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