
#include <errno.h>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <fstream>
#include <iostream>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include "fastslam_core.hpp"

#include <random>

#include <ros/ros.h>

using namespace std;
using namespace Eigen;

float calculate_covariance(vector<float> &X, vector<float> &Y) {

  if (X.size() != Y.size()) {
    throw invalid_argument("Size of X must equal size of Y");
  }

  unsigned int N = X.size();

  float meanX = reduce(X.begin(), X.end()) / static_cast<float>(N);
  float meanY = reduce(Y.begin(), Y.end()) / static_cast<float>(N);

  float covariance = 0.0;

  for (unsigned int i = 0; i < N; i++) {
    covariance += (X[i] - meanX) * (Y[i] - meanY);
  }

  return covariance / static_cast<float>(N);
}

float calculate_covariance(vector<float> &X, vector<float> &Y,
                           vector<float> &P) {

  if (X.size() != Y.size() || X.size() != P.size()) {
    throw invalid_argument("Size of X must equal size of Y and P");
  }

  unsigned int N = X.size();

  float meanX = 0.0;
  float meanY = 0.0;

  for (unsigned int i = 0; i < N; i++) {
    meanX += X[i] * P[i];
    meanY += Y[i] * P[i];
  }

  meanX /= static_cast<float>(N);
  meanY /= static_cast<float>(N);

  float covariance = 0.0;

  for (unsigned int i = 0; i < N; i++) {
    covariance += (X[i] - meanX) * (Y[i] - meanY) * P[i];
  }

  return covariance / static_cast<float>(N);
}

inline float sqr(float x) { return x * x; }

void add_control_noise(float V, float G, MatrixXf &Q, int addnoise,
                       float *VnGn) {
  if (addnoise == 1) {
    VectorXf A(2);
    A(0) = V;
    A(1) = G;
    VectorXf C(2);
    C = multivariate_gauss(A, Q, 1);
    VnGn[0] = C(0);
    VnGn[1] = C(1);
  }
}

void predict_true(VectorXf &xv, // cppcheck-suppress constParameter
                  float V, float G, float WB, float dt) {
  xv(0) = xv(0) + V * dt * cos(G + xv(2));
  xv(1) = xv(1) + V * dt * sin(G + xv(2));
  xv(2) = pi_to_pi(xv(2) + V * dt * sin(G) / WB);
}

void compute_steering(const VectorXf &x, MatrixXf &wp, int &iwp, float minD,
                      float &G, float rateG, float maxG, float dt) {
  /*
      % INPUTS:
      %   x - true position
      %   wp - waypoints
      %   iwp - index to current waypoint
      %   minD - minimum distance to current waypoint before switching to next
      %   rateG - max steering rate (rad/s)
      %   maxG - max steering angle (rad)
      %   dt - timestep
      % Output:
      %   G - current steering angle
  */

  // determine if current waypoint reached
  Vector2d cwp;
  cwp[0] = wp(0, iwp); //-1 since indexed from 0
  cwp[1] = wp(1, iwp);

  float d2 = pow((cwp[0] - x[0]), 2) + pow((cwp[1] - x[1]), 2);

  if (d2 < minD * minD) {
    iwp++; // switch to next
    if (iwp >= wp.cols()) {
      iwp = -1;
      return;
    }

    cwp[0] = wp(0, iwp); //-1 since indexed from 0
    cwp[1] = wp(1, iwp);
  }

  // compute change in G to point towards current waypoint
  float deltaG = atan2(cwp[1] - x[1], cwp[0] - x[0]) - x[2] - G;
  deltaG = pi_to_pi(deltaG);

  // limit rate
  float maxDelta = rateG * dt;
  if (abs(deltaG) > maxDelta) {
    int sign = (deltaG > 0) ? 1 : ((deltaG < 0) ? -1 : 0);
    deltaG = sign * maxDelta;
  }

  // limit angle
  G = G + deltaG;
  if (abs(G) > maxG) {
    int sign2 = (G > 0) ? 1 : ((G < 0) ? -1 : 0);
    G = sign2 * maxG;
  }
}

// z is range and bearing of visible landmarks
//  find associations (zf) and new features (zn)
void data_associate_known(const vector<VectorXf> &z, const vector<int> &idz,
                          VectorXf &table, // cppcheck-suppress constParameter
                          vector<VectorXf> &zf, int Nf, vector<int> &idf,
                          vector<VectorXf> &zn) {
  idf.clear();
  vector<int> idn;

  for (unsigned long i = 0; i < idz.size(); i++) {
    unsigned long ii = idz[i];
    VectorXf z_i;
    if (table(ii) == -1) { // new feature
      z_i = z[i];
      zn.push_back(z_i);
      idn.push_back(ii);
    } else {
      z_i = z[i];
      zf.push_back(z_i);
      idf.push_back(table(ii));
    }
  }

  assert(idn.size() == zn.size());
  for (unsigned long i = 0; i < idn.size(); i++) {
    table(idn[i]) = Nf + i;
  }
}

// z is the list of measurements conditioned on the particle.
void feature_update(Particle &particle, const vector<VectorXf> &z,
                    vector<int> &idf, const MatrixXf &R,
                    const vector<int> &observationClass,
                    const vector<float> &beliefs, const vector<VectorXf> &zp,
                    vector<MatrixXf> &Hv, const vector<MatrixXf> &Hf,
                    vector<MatrixXf> &Sf) {
  // Having selected a new pose from the proposal distribution,
  //   this pose is assumed perfect and each feature update maybe
  //   computed independently and without pose uncertainty
  vector<VectorXf> xf; // updated EKF means
  vector<MatrixXf> Pf; // updated EKF covariances

  for (unsigned long i = 0; i < idf.size(); i++) {
    xf.push_back(particle.xf()[idf[i]]); // means
    Pf.push_back(particle.Pf()[idf[i]]); // covariances
  }

  vector<VectorXf> v; // difference btw two measurements (used to update mean)
  for (unsigned long i = 0; i < z.size(); i++) {
    VectorXf measure_diff = z[i] - zp[i];
    measure_diff[1] = pi_to_pi(measure_diff[1]);
    v.push_back(measure_diff);
  }

  VectorXf vi;
  MatrixXf Hfi;
  MatrixXf Pfi;
  VectorXf xfi;

  for (unsigned long i = 0; i < idf.size(); i++) {
    vi = v[i];
    Hfi = Hf[i];
    Pfi = Pf[i];
    xfi = xf[i];
    KF_cholesky_update(xfi, Pfi, vi, R, Hfi);
    xf[i] = xfi;
    Pf[i] = Pfi;
  }

  for (unsigned long i = 0; i < idf.size(); i++) {
    particle.setXfi(idf[i], xf[i]);
    particle.setPfi(idf[i], Pf[i]);

    LandmarkMetadata old_meta = particle.metadata()[idf[i]];

    old_meta.score++;
    old_meta.classDetectionCount[observationClass[i]] += beliefs[i];

    particle.setMetadatai(idf[i], old_meta);
  }
}

vector<VectorXf> get_observations(const VectorXf &x, MatrixXf lm,
                                  vector<int> &idf, float rmax) {
  get_visible_landmarks(x, lm, idf, rmax);
  return compute_range_bearing(x, lm);
}

void get_visible_landmarks(const VectorXf &x, MatrixXf &lm, vector<int> &idf,
                           float rmax) {
  // select set of landmarks that are visible within vehicle's
  // semi-circular field of view
  vector<float> dx;
  vector<float> dy;

  for (int c = 0; c < lm.cols(); c++) {
    dx.push_back(lm(0, c) - x(0));
    dy.push_back(lm(1, c) - x(1));
  }

  float phi = x(2);

  // distant points are eliminated
  vector<int> ii = find2(dx, dy, phi, rmax);

  MatrixXf lm_new(lm.rows(), ii.size());
  unsigned j, k;
  for (j = 0; j < lm.rows(); j++) {
    for (k = 0; k < ii.size(); k++) {
      lm_new(j, k) = lm(j, ii[k]);
    }
  }
  lm = MatrixXf(lm_new);

  vector<int> idf_backup(idf);
  idf.clear();
  for (unsigned long i = 0; i < ii.size(); i++) {
    idf.push_back(idf_backup[ii[i]]);
  }
}

vector<VectorXf> compute_range_bearing(const VectorXf &x, MatrixXf &lm) {
  vector<float> dx;
  vector<float> dy;

  for (int c = 0; c < lm.cols(); c++) {
    dx.push_back(lm(0, c) - x(0));
    dy.push_back(lm(1, c) - x(1));
  }

  assert(dx.size() == lm.cols());
  assert(dy.size() == lm.cols());

  float phi = x(2);
  vector<VectorXf> z;

  for (int i = 0; i < lm.cols(); i++) {
    VectorXf zvec(2);
    zvec(0) = sqrt(pow(dx[i], 2) + pow(dy[i], 2));
    zvec(1) = atan2(dy[i], dx[i]) - phi;
    z.push_back(zvec);
  }

  return z;
}

vector<int> find2(vector<float> &dx, vector<float> &dy, float phi, float rmax) {
  vector<int> index;
  // incremental tests for bounding semi-circle
  for (unsigned long j = 0; j < dx.size(); j++) {
    if ((abs(dx[j]) < rmax) && (abs(dy[j]) < rmax) &&
        ((dx[j] * cos(phi) + dy[j] * sin(phi)) > 0.0) &&
        (((float)pow(dx[j], 2) + (float)pow(dy[j], 2)) < (float)pow(rmax, 2))) {
      index.push_back(j);
    }
  }
  return index;
}

void KF_cholesky_update(VectorXf &x, MatrixXf &P, const VectorXf &v,
                        const MatrixXf &R, MatrixXf &H) {
  MatrixXf PHt = P * H.transpose();
  MatrixXf S = H * PHt + R;

  // FIXME: why use conjugate()?
  S = (S + S.transpose()) * 0.5; // make symmetric
  MatrixXf SChol = S.llt().matrixU();
  // SChol.transpose();
  // SChol.conjugate();

  MatrixXf SCholInv = SChol.inverse(); // tri matrix
  MatrixXf W1 = PHt * SCholInv;
  MatrixXf W = W1 * SCholInv.transpose();

  x = x + W * v;
  P = P - W1 * W1.transpose();
}

void KF_joseph_update(VectorXf &x, MatrixXf &P, float v, float R, MatrixXf &H) {
  VectorXf PHt = P * H.transpose();
  MatrixXf S = H * PHt;
  MatrixXf _t = S;
  _t.setOnes();
  _t = _t * R;
  S = S + _t;
  MatrixXf Si = S.inverse();
  // Si = make_symmetric(Si);
  make_symmetric(Si);

  VectorXf W = PHt * Si;
  x = x + W * v;

  // Joseph-form covariance update
  MatrixXf eye(P.rows(), P.cols());
  eye.setIdentity();
  MatrixXf C = eye - W * H;
  P = C * P * C.transpose() + W * R * W.transpose();

  float eps = 2.2204 * pow(10.0, -16); // numerical safety
  P = P + eye * eps;

  MatrixXf PSD_check = P.llt().matrixL();
  PSD_check.transpose();
  PSD_check.conjugate(); // for upper tri
}

MatrixXf make_symmetric(MatrixXf &P) { return (P + P.transpose()) * 0.5; }

MatrixXf line_plot_conversion(MatrixXf &lnes) {
  int len = lnes.cols() * 3 - 1;
  MatrixXf p(2, len);

  for (int j = 0; j < len; j += 3) {
    int k = floor((j + 1) / 3); // reverse of len calculation
    p(0, j) = lnes(0, k);
    p(1, j) = lnes(1, k);
    p(0, j + 1) = lnes(2, k);
    p(1, j + 1) = lnes(3, k);
    if (j + 2 < len) {
      p(0, j + 2) = 0;
      p(1, j + 2) = 0;
    }
  }
  return p;
}

// rb is measurements
// xv is robot pose
MatrixXf make_laser_lines(const vector<VectorXf> &rb, const VectorXf &xv) {
  if (rb.empty()) {
    return MatrixXf(0, 0);
  }

  unsigned long len = rb.size();
  MatrixXf lnes(4, len);

  MatrixXf globalMat(2, rb.size());
  int j;
  for (j = 0; j < globalMat.cols(); j++) {
    globalMat(0, j) = rb[j][0] * cos(rb[j][1]);
    globalMat(1, j) = rb[j][0] * sin(rb[j][1]);
  }

  TransformToGlobal(globalMat, xv);

  for (int c = 0; c < lnes.cols(); c++) {
    lnes(0, c) = xv(0);
    lnes(1, c) = xv(1);
    lnes(2, c) = globalMat(0, c);
    lnes(3, c) = globalMat(1, c);
  }

  // return line_plot_conversion(lnes);
  return lnes;
}

void make_covariance_ellipse(const MatrixXf &x, MatrixXf &P, MatrixXf &lines) {
  int i, N;
  float p_inc;
  float s;
  MatrixXf r;

  N = 16;
  p_inc = 2.0 * M_PI / N;
  s = 2.0;

  lines.setZero(2, N + 1);

  r = P.sqrt();

  for (i = 0; i <= N; i++) {
    float p = i * p_inc;
    float cp = cos(p);
    float sp = sin(p);

    lines(0, i) = x(0) + s * (r(0, 0) * cp + r(0, 1) * sp);
    lines(1, i) = x(1) + s * (r(1, 0) * cp + r(1, 1) * sp);
  }
}

// http://moby.ihme.washington.edu/bradbell/mat2cpp/randn.cpp.xml
MatrixXf nRandMat::randn(int m, int n) {
  // use formula 30.3 of Statistical Distributions (3rd ed)
  // Merran Evans, Nicholas Hastings, and Brian Peacock
  int urows = m * n + 1;
  VectorXf u(urows);

  // u is a random matrix
#if 1
  for (int r = 0; r < urows; r++) {
    // FIXME: better way?
    u(r) = std::rand() * 1.0 / RAND_MAX;
  }
#else
  u = ((VectorXf::Random(urows).array() + 1.0) / 2.0).matrix();
#endif

  MatrixXf x(m, n);

  int i, j, k;
  float square, angle;
  float amp = 0.;
  k = 0;
  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++) {
      if (k % 2 == 0) {
        square = -2. * std::log(u(k));
        if (square < 0.)
          square = 0.;
        amp = sqrt(square);
        angle = 2. * M_PI * u(k + 1);
        x(i, j) = amp * std::sin(angle);
      } else
        x(i, j) = amp * std::cos(angle);

      k++;
    }
  }

  return x;
}

MatrixXf nRandMat::rand(int m, int n) {
  MatrixXf x(m, n);
  int i, j;
  float rand_max = float(RAND_MAX);

  for (i = 0; i < m; i++) {
    for (j = 0; j < n; j++)
      x(i, j) = float(std::rand()) / rand_max;
  }
  return x;
}

// add random measurement noise. We assume R is diagnoal matrix
void add_observation_noise(vector<VectorXf> &z, const MatrixXf &R,
                           int addnoise) {
  if (addnoise == 1) {
    unsigned long len = z.size();
    if (len > 0) {
      MatrixXf randM1 = nRandMat::randn(1, len);
      MatrixXf randM2 = nRandMat::randn(1, len);

      for (unsigned long c = 0; c < len; c++) {
        z[c][0] = z[c][0] + randM1(0, c) * sqrt(R(0, 0));
        z[c][1] = z[c][1] + randM2(0, c) * sqrt(R(1, 1));
      }
    }
  }
}

VectorXf multivariate_gauss(VectorXf &x, MatrixXf &P, int n) {
  int len = x.size();

  // choleksy decomposition
  MatrixXf S = P.llt().matrixL();
  MatrixXf X = nRandMat::randn(len, n);

  return S * X + x;
}

void pi_to_pi(VectorXf &angle) {
  int i, n;

  for (i = 0; i < angle.size(); i++) {
    if ((angle[i] < (-2 * M_PI)) || (angle[i] > (2 * M_PI))) {
      n = floor(angle[i] / (2 * M_PI));
      angle[i] = angle[i] - n * (2 * M_PI);
    }

    if (angle[i] > M_PI)
      angle[i] = angle[i] - (2 * M_PI);

    if (angle[i] < -M_PI)
      angle[i] = angle[i] + (2 * M_PI);
  }
}

float pi_to_pi(float ang) {
  if ((ang < -2 * M_PI) || (ang > 2 * M_PI)) {
    int n = floor(ang / (2 * M_PI));
    ang = ang - n * (2 * M_PI);
  }

  if (ang > M_PI)
    ang = ang - (2 * M_PI);

  if (ang < -M_PI)
    ang = ang + (2 * M_PI);

  return ang;
}

float pi_to_pi2(float ang) {
  if (ang > M_PI)
    ang = ang - (2 * M_PI);

  if (ang < -M_PI)
    ang = ang + (2 * M_PI);

  return ang;
}

//
// add new features
//
void add_feature(Particle &particle, const vector<VectorXf> &z, MatrixXf &R,
                 const vector<int> &observationClass,
                 const vector<float> &beliefs) {
  int lenz = z.size();
  vector<VectorXf> xf;
  vector<MatrixXf> Pf;
  VectorXf xv = particle.xv();

  MatrixXf Gz(2, 2);

  for (int i = 0; i < lenz; i++) {
    float r = z[i][0];
    float b = z[i][1];
    float s = sin(xv(2) + b);
    float c = cos(xv(2) + b);

    VectorXf measurement(2);
    measurement(0) = xv(0) + r * c;
    measurement(1) = xv(1) + r * s;
    xf.push_back(measurement);
    Gz << c, -r * s, s, r * c;

    Pf.push_back(Gz * R * Gz.transpose());
  }

  int lenx = particle.xf().size();

  for (int i = 0; i < lenz; i++) {
    particle.setXfi(i + lenx, xf[(i)]);
    particle.setPfi(i + lenx, Pf[(i)]);
    LandmarkMetadata meta;
    meta.classDetectionCount[observationClass[i]] = beliefs[i];
    particle.setMetadatai(i + lenx, meta);
  }
}

void compute_jacobians(
    Particle &particle, vector<int> &idf, const MatrixXf &R,
    vector<VectorXf> &zp, // measurement (range, bearing)
    vector<MatrixXf> *Hv, // jacobians of function h (deriv of h wrt pose)
    vector<MatrixXf> *Hf, // jacobians of function h (deriv of h wrt mean)
    vector<MatrixXf> *Sf) // measurement covariance
{
  VectorXf xv = particle.xv();

  vector<VectorXf> xf;
  vector<MatrixXf> Pf;

  unsigned int i;
  for (i = 0; i < idf.size(); i++) {
    xf.push_back(particle.xf()[idf[i]]);
    Pf.push_back((particle.Pf())[idf[i]]); // particle.Pf is a array of matrices
  }

  MatrixXf HvMat(2, 3);
  MatrixXf HfMat(2, 2);

  for (i = 0; i < idf.size(); i++) {
    float dx = xf[i](0) - xv(0);
    float dy = xf[i](1) - xv(1);
    float d2 = pow(dx, 2) + pow(dy, 2);
    float d = sqrt(d2);

    VectorXf zp_vec(2);

    // predicted observation
    zp_vec[0] = d;
    zp_vec[1] = pi_to_pi(atan2(dy, dx) - xv(2));
    zp.push_back(zp_vec);

    // Jacobian wrt vehicle states
    HvMat(0, 0) = -dx / d;
    HvMat(1, 0) = -dy / d;
    HvMat(0, 1) = 0;
    HvMat(1, 1) = dy / d2;
    HvMat(0, 2) = -dx / d2;
    HvMat(1, 2) = -1;

    // Jacobian wrt feature states
    HfMat(0, 0) = dx / d;
    HfMat(1, 0) = dy / d;
    HfMat(0, 1) = -dy / d2;
    HfMat(1, 1) = dx / d2;

    Hv->push_back(HvMat);
    Hf->push_back(HfMat);

    // innovation covariance of feature observation given the vehicle'
    MatrixXf SfMat = HfMat * Pf[i] * HfMat.transpose() + R;
    Sf->push_back(SfMat);
  }
}

void resample_particles(vector<Particle> &particles, int Nmin, int doresample) {
  unsigned long i;
  unsigned long N = particles.size();
  VectorXf w(N);

  for (i = 0; i < N; i++) {
    w(i) = particles[i].w();
  }

  float ws = w.sum();
  for (i = 0; i < N; i++) {
    particles[i].setW(w(i) / ws);
  }

  float Neff = 0.0;
  vector<int> keep;

  stratified_resample(w, keep, Neff);

  vector<Particle> old_particles = vector<Particle>(particles);
  particles.resize(keep.size());

  if ((Neff < Nmin) && (doresample == 1)) {
    for (i = 0; i < keep.size(); i++) {
      particles[i] = old_particles[keep[i]];
    }

    for (i = 0; i < N; i++) {
      float new_w = 1.0f / (float)N;
      particles[i].setW(new_w);
    }
  }
}

void stratified_random(unsigned long N, vector<float> &di) {
  float k = 1.0 / (float)N;

  // deterministic intervals
  float temp = k / 2;
  while (temp < (1 - k / 2)) {
    di.push_back(temp);
    temp = temp + k;
  }

  // FIXME: when set NPARTICLES = 30, this whill failed
  assert(di.size() == N);

  // dither within interval
  vector<float>::iterator diter;
  for (diter = di.begin(); diter != di.end(); ++diter) {
    *diter = (*diter) + unifRand() * k - (k / 2);
  }
}

//
// Generate a random number between 0 and 1
// return a uniform number in [0,1].
double unifRand() { return rand() / double(RAND_MAX); }

// FIXME: input w will be modified?
void stratified_resample(VectorXf w, vector<int> &keep, float &Neff) {
  VectorXf wsqrd(w.size());
  double w_sum = w.sum();

  for (int i = 0; i < w.size(); i++) {
    // FIXME: matlab version is: w = w / sum(w)
    w(i) = w(i) / w_sum;
    wsqrd(i) = pow(w(i), 2);
  }
  Neff = 1.0 / (float)wsqrd.sum();

  int len = w.size();
  keep.resize(len);
  for (int i = 0; i < len; i++) {
    keep[i] = i;
  }

  vector<float> select;
  stratified_random(len, select);
  cumsum(w);

  int ctr = 0;
  for (int i = 0; i < len; i++) {
    while ((ctr < len) && (select[ctr] < w(i))) {
      keep[ctr] = i;
      ctr++;
    }
  }
}

//
// returns a cumulative sum array
//
void cumsum(VectorXf &w) {
  VectorXf csumVec(w.size());
  for (int i = 0; i < w.size(); i++) {
    float sum = 0;
    for (int j = 0; j <= i; j++) {
      sum += w(j);
    }
    csumVec(i) = sum;
  }

  w = VectorXf(csumVec); // copy constructor. Double check
}

void TransformToGlobal(MatrixXf &p, const VectorXf &b) {
  // rotate
  MatrixXf rot(2, 2);
  rot << cos(b(2)), -sin(b(2)), sin(b(2)), cos(b(2));

  MatrixXf p_resized;
  p_resized = MatrixXf(p);
  p_resized.conservativeResize(2, p_resized.cols());
  p_resized = rot * p_resized;

  // translate
  int c;
  for (c = 0; c < p_resized.cols(); c++) {
    p(0, c) = p_resized(0, c) + b(0);
    p(1, c) = p_resized(1, c) + b(1);
  }

  // if p is a pose and not a point
  if (p.rows() == 3) {
    for (int k = 0; k < p_resized.cols(); k++) {
      float input = p(2, k) + b(2);
      p(2, k) = pi_to_pi(input);
    }
  }
}

void read_slam_input_file(const string &s, MatrixXf *lm, MatrixXf *wp) {
  if (access(s.c_str(), R_OK) == -1) {
    std::cerr << "Unable to read input file" << s << std::endl;
    exit(EXIT_FAILURE);
  }

  ifstream in(s.c_str());

  int lineno = 0;
  unsigned long lm_rows = 0;
  int lm_cols = 0;
  unsigned long wp_rows = 0;
  unsigned long wp_cols = 0;

  while (in) {
    lineno++;
    string str;
    getline(in, str);
    istringstream line(str);

    vector<string> tokens;
    copy(istream_iterator<string>(line), istream_iterator<string>(),
         back_inserter<vector<string>>(tokens));

    if (tokens.size() == 0) {
      continue;
    } else if (tokens[0][0] == '#') {
      continue;
    } else if (tokens[0] == "lm") {
      if (tokens.size() != 3) {
        std::cerr << "Wrong args for lm!" << std::endl;
        std::cerr << "Error occuredon line" << lineno << std::endl;
        std::cerr << "line:" << str << std::endl;
        exit(EXIT_FAILURE);
      }
      lm_rows = strtof(tokens[1].c_str(), NULL);
      lm_cols = strtof(tokens[2].c_str(), NULL);

      lm->resize(lm_rows, lm_cols);
      for (int c = 0; c < lm_cols; c++) {
        lineno++;
        if (!in) {
          std::cerr << "EOF after reading" << std::endl;
          exit(EXIT_FAILURE);
        }
        getline(in, str);
        istringstream line1(str);
        vector<string> tokens1;
        copy(istream_iterator<string>(line1), istream_iterator<string>(),
             back_inserter<vector<string>>(tokens1));
        if (tokens1.size() < lm_rows) {
          std::cerr << "invalid line for lm coordinate!" << std::endl;
          std::cerr << "Error occured on line " << lineno << std::endl;
          std::cerr << "line: " << str << std::endl;
          exit(EXIT_FAILURE);
        }

        for (unsigned long r = 0; r < lm_rows; r++) {
          (*lm)(r, c) = strtof(tokens1[r].c_str(), NULL);
        }
      }
    } else if (tokens[0] == "wp") {
      if (tokens.size() != 3) {
        std::cerr << "Wrong args for wp!" << std::endl;
        std::cerr << "Error occured on line" << lineno << std::endl;
        std::cerr << "line:" << str << std::endl;
        exit(EXIT_FAILURE);
      }
      wp_rows = strtof(tokens[1].c_str(), NULL);
      wp_cols = strtof(tokens[2].c_str(), NULL);
      wp->resize(wp_rows, wp_cols);
      for (unsigned long c = 0; c < wp_cols; c++) {
        lineno++;
        if (!in) {
          std::cerr << "EOF after reading" << std::endl;
          exit(EXIT_FAILURE);
        }
        getline(in, str);
        istringstream line2(str);
        std::vector<string> tokens2;
        copy(istream_iterator<string>(line2), istream_iterator<string>(),
             back_inserter<std::vector<string>>(tokens2));
        if (tokens2.size() < wp_rows) {
          std::cerr << "invalid line for wp coordinate!" << std::endl;
          std::cerr << "Error occured on line " << lineno << std::endl;
          std::cerr << "line: " << str << std::endl;
          exit(EXIT_FAILURE);
        }

        for (unsigned long r = 0; r < lm_rows; r++) {
          (*wp)(r, c) = strtof(tokens2[r].c_str(), NULL);
        }
      }
    } else {
      std::cerr << "Unkwown command" << tokens[0] << std::endl;
      std::cerr << "Error occured on line" << lineno << std::endl;
      std::cerr << "line: " << str << std::endl;
      exit(EXIT_FAILURE);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// particle class
////////////////////////////////////////////////////////////////////////////////
Particle::Particle(double stand_dev) {
  _w = 1.0;
  _xv = VectorXf(3);
  _xv.setZero(3);
  _prev_yaw = 0.0;
  _rev = 0;
  _Pv = MatrixXf(3, 3);
  _Pv.setZero(3, 3);
  _da = NULL;

  std::random_device rd{};
  std::mt19937 gen{rd()};

  std::normal_distribution d{0.0, stand_dev};

  _xv(0) = d(gen);
  _xv(1) = d(gen);
}

Particle::Particle() {
  _w = 1.0;
  _xv = VectorXf(3);
  _xv.setZero(3);
  _prev_yaw = 0.0;
  _rev = 0;
  _Pv = MatrixXf(3, 3);
  _Pv.setZero(3, 3);
  _da = NULL;
}

Particle::Particle(float &w, VectorXf &xv, MatrixXf &Pv, vector<VectorXf> &xf,
                   vector<MatrixXf> &Pf, float *da) {
  _w = w;
  _xv = xv;
  _prev_yaw = 0.0;
  _rev = 0;
  _Pv = Pv;
  _xf = xf;
  _Pf = Pf;
  _da = da;
}

Particle::~Particle() {
  this->_xf.clear();
  this->_Pf.clear();
  this->_metadata.clear();
}

// getters
float &Particle::w() { return _w; }

VectorXf &Particle::xv() { return _xv; }

float &Particle::prevyaw() { return _prev_yaw; }

int &Particle::rev() { return _rev; }

MatrixXf &Particle::Pv() { return _Pv; }

vector<VectorXf> &Particle::xf() { return _xf; }

vector<LandmarkMetadata> &Particle::metadata() { return _metadata; }

vector<MatrixXf> &Particle::Pf() { return _Pf; }

float *Particle::da() { return _da; }

// setters
void Particle::setW(float w) { _w = w; }

void Particle::setXv(VectorXf &xv) { _xv = xv; }

void Particle::setPrevyaw(float yaw) { _prev_yaw = yaw; }

void Particle::setRev(int rev) { _rev = rev; }

void Particle::incRev() { _rev++; }

void Particle::decRev() { _rev--; }

void Particle::setPv(MatrixXf &Pv) { _Pv = Pv; }

void Particle::setXf(vector<VectorXf> &xf) { _xf = xf; }

void Particle::setXfi(unsigned long i, VectorXf &vec) {
  if (i >= _xf.size()) {
    _xf.resize(i + 1);
  }
  _xf[i] = vec;
}

void Particle::setMetadata(vector<LandmarkMetadata> &meta) { _metadata = meta; }

void Particle::setMetadatai(unsigned long i, LandmarkMetadata &meta) {
  if (i >= _metadata.size()) {
    _metadata.resize(i + 1);
  }

  for (int j = 0; j < LANDMARK_CLASS_COUNT; j++) {
    _metadata[i].classDetectionCount[j] = meta.classDetectionCount[j];
  }
  _metadata[i].score = meta.score;
}

void Particle::setPf(vector<MatrixXf> &Pf) { _Pf = Pf; }

void Particle::setPfi(unsigned long i, MatrixXf &m) {
  if (i >= _Pf.size()) {
    _Pf.resize(i + 1);
  }
  _Pf[i] = m;
}

void Particle::setDa(float *da) { _da = da; }

////////////////////////////////////////////////////////////////////////////////
// EKF-SLAM functions
////////////////////////////////////////////////////////////////////////////////

void ekf_predict(VectorXf &x, // cppcheck-suppress constParameter
                 MatrixXf &P, float V, float G, const MatrixXf &Q, float WB,
                 float dt) {
  float s, c;
  float vts, vtc;
  MatrixXf Gv(3, 3), Gu(3, 2); // Jacobians

  s = sin(G + x(2));
  c = cos(G + x(2));
  vts = V * dt * s;
  vtc = V * dt * c;

  // Gv initialization
  Gv(0, 0) = 1;
  Gv(0, 1) = 0;
  Gv(0, 2) = -vts;

  Gv(1, 0) = 0;
  Gv(1, 1) = 1;
  Gv(1, 2) = vtc;

  Gv(2, 0) = 0;
  Gv(2, 1) = 0;
  Gv(2, 2) = 1;

  // Gu initialization
  Gu(0, 0) = dt * c;
  Gu(0, 1) = -vts;
  Gu(0, 2) = dt * s;

  Gu(1, 0) = vtc;
  Gu(1, 1) = dt * sin(G) / WB;
  Gu(1, 2) = V * dt * cos(G) / WB;

  // predict covariance
  //      P(1:3,1:3)= Gv*P(1:3,1:3)*Gv' + Gu*Q*Gu';
  P.block(0, 0, 3, 3) =
      Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gu * Q * Gu.transpose();

  int m = P.rows();
  if (m > 3) {
    P.block(0, 3, 3, m - 3) = Gv * P.block(0, 3, 3, m - 3);
    P.block(3, 0, m - 3, 3) = P.block(0, 3, 3, m - 3).transpose();
  }

  // predict state
  x(0) = x(0) + vtc;
  x(1) = x(1) + vts;
  x(2) = pi_to_pi(x(2) + V * dt * sin(G) / WB);
}

void ekf_observe_heading(VectorXf &x, MatrixXf &P, float phi, int useheading,
                         float sigmaPhi) {
  if (useheading == 0)
    return;

  float v;
  MatrixXf H = MatrixXf(1, x.size());

  H.setZero(1, x.size());
  H(2) = 1;
  v = pi_to_pi(phi - x(2));

  KF_joseph_update(x, P, v, sigmaPhi * sigmaPhi, H);
}

void ekf_observe_model(VectorXf &x, int idf, VectorXf &z, MatrixXf &H) {
  int Nxv, fpos;
  float dx, dy, d2, d, xd, yd, xd2, yd2;

  Nxv = 3;
  H.setZero(2, x.size());
  z.setZero(2);

  // position of xf in state
  fpos = Nxv + idf * 2;

  dx = x(fpos) - x(0);
  dy = x(fpos + 1) - x(1);
  d2 = sqr(dx) + sqr(dy);
  d = sqrtf(d2);
  xd = dx / d;
  yd = dy / d;
  xd2 = dx / d2;
  yd2 = dy / d2;

  z(0) = d;
  z(1) = atan2(dy, dx) - x(2);

  H(0, 0) = -xd;
  H(0, 1) = -yd;
  H(0, 2) = 0;
  H(1, 0) = yd2;
  H(1, 1) = -xd2;
  H(1, 2) = -1;

  H(0, fpos) = xd;
  H(0, fpos + 1) = yd;
  H(1, fpos) = -yd2;
  H(1, fpos + 1) = xd2;
}

void ekf_compute_association(VectorXf &x, const MatrixXf &P, const VectorXf &z,
                             const MatrixXf &R, int idf, float &nis,
                             float &nd) {
  VectorXf zp;
  MatrixXf H;
  VectorXf v(2);
  MatrixXf S;

  ekf_observe_model(x, idf, zp, H);

  v = z - zp;
  v(1) = pi_to_pi(v(1));

  S = H * P * H.transpose() + R;

  nis = v.transpose() * S.inverse() * v;
  nd = nis + log(S.determinant());
}

void ekf_data_associate(VectorXf &x, const MatrixXf &P, vector<VectorXf> &z,
                        const MatrixXf &R, float gate1, float gate2,
                        vector<VectorXf> &zf, vector<int> &idf,
                        vector<VectorXf> &zn) {
  unsigned long Nxv, Nf;
  unsigned long i, j;
  float nis, nd;

  zf.clear();
  zn.clear();
  idf.clear();

  Nxv = 3;                   // number of vehicle pose states
  Nf = (x.size() - Nxv) / 2; // number of features already in map

  // linear search for nearest-neighbour, no clever tricks (like a quick
  // bounding-box threshold to remove distant features; or, better yet,
  // a balanced k-d tree lookup). TODO: implement clever tricks.
  for (i = 0; i < z.size(); i++) {
    long jbest = -1;
    float nbest = 1e60;
    float outer = 1e60;

    for (j = 0; j < Nf; j++) {
      ekf_compute_association(x, P, z[i], R, j, nis, nd);

      if (nis < gate1 && nd < nbest) {
        nbest = nd;
        jbest = j;
      } else if (nis < outer) {
        outer = nis;
      }
    }

    if (jbest > -1) {
      // add nearest-neighbour to association list
      zf.push_back(z[i]);
      idf.push_back(jbest);
    } else if (outer > gate2) {
      // z too far to associate, but far enough to be a new feature
      zn.push_back(z[i]);
    }
  }
}

// z is range and bearing of visible landmarks
//  find associations (zf) and new features (zn)
void ekf_data_associate_known(VectorXf &x, const vector<VectorXf> &z,
                              const vector<int> &idz, vector<VectorXf> &zf,
                              vector<int> &idf, vector<VectorXf> &zn,
                              vector<int> &table) {
  vector<int> idn;
  unsigned i;

  zf.clear();
  zn.clear();
  idf.clear();
  idn.clear();

  for (i = 0; i < idz.size(); i++) {
    unsigned ii = idz[i];
    VectorXf z_i;

    if (table[ii] == -1) {
      // new feature
      z_i = z[i];
      zn.push_back(z_i);
      idn.push_back(ii);
    } else {
      // exist feature
      z_i = z[i];
      zf.push_back(z_i);
      idf.push_back(table[ii]);
    }
  }

  // add new feature IDs to lookup table
  int Nxv = 3;                   // number of vehicle pose states
  int Nf = (x.size() - Nxv) / 2; // number of features already in map

  // add new feature positions to lookup table
  //      table[idn]= Nf + (1:size(zn,2));
  for (i = 0; i < idn.size(); i++) {
    table[idn[i]] = Nf + i;
  }
}

void ekf_batch_update(VectorXf &x, MatrixXf &P, const vector<VectorXf> &zf,
                      const MatrixXf &R, const vector<int> &idf) {
  int lenz, lenx;
  MatrixXf H, RR;
  VectorXf v;
  VectorXf zp;
  MatrixXf H_;

  int i;

  lenz = zf.size();
  lenx = x.size();

  H.setZero(2 * lenz, lenx);
  v.setZero(2 * lenz);
  RR.setZero(2 * lenz, 2 * lenz);

  zp.setZero(2);

  for (i = 0; i < lenz; i++) {
    ekf_observe_model(x, idf[i], zp, H_);
    H.block(i * 2, 0, 2, lenx) = H_;

    v(2 * i) = zf[i](0) - zp(0);
    v(2 * i + 1) = pi_to_pi(zf[i](1) - zp(1));

    RR.block(i * 2, i * 2, 2, 2) = R;
  }

  KF_cholesky_update(x, P, v, RR, H);
}

void ekf_update(VectorXf &x, MatrixXf &P, const vector<VectorXf> &zf,
                const MatrixXf &R, const vector<int> &idf, int batch) {
  if (batch == 1)
    ekf_batch_update(x, P, zf, R, idf);
}

void ekf_add_one_z(VectorXf &x, MatrixXf &P, const VectorXf &z,
                   const MatrixXf &Re) {
  int len;
  float r, b;
  float s, c;

  MatrixXf Gv, Gz;

  len = x.size();

  r = z(0);
  b = z(1);
  s = sin(x(2) + b);
  c = cos(x(2) + b);

  // augment x
  //      x = [x; x(1)+r*c; x(2)+r*s];
  x.conservativeResize(len + 2);
  x(len) = x(0) + r * c;
  x(len + 1) = x(1) + r * s;

  // Jacobians
  Gv.setZero(2, 3);
  Gz.setZero(2, 2);

  Gv << 1, 0, -r * s, 0, 1, r * c;
  Gz << c, -r * s, s, r * c;

  // augment P
  P.conservativeResize(len + 2, len + 2);

  // feature cov
  //      P(rng,rng)= Gv*P(1:3,1:3)*Gv' + Gz*R*Gz';
  P.block(len, len, 2, 2) =
      Gv * P.block(0, 0, 3, 3) * Gv.transpose() + Gz * Re * Gz.transpose();

  // vehicle to feature xcorr
  //      P(rng,1:3)= Gv*P(1:3,1:3);
  //      P(1:3,rng)= P(rng,1:3)';
  P.block(len, 0, 2, 3) = Gv * P.block(0, 0, 3, 3);
  P.block(0, len, 3, 2) = P.block(len, 0, 2, 3).transpose();

  if (len > 3) {
    // map to feature xcoor
    //   P(rng,rnm)= Gv*P(1:3,rnm);
    P.block(len, 3, 2, len - 3) = Gv * P.block(0, 3, 3, len - 3);

    //   P(rnm,rng)= P(rng,rnm)';
    P.block(3, len, len - 3, 2) = P.block(len, 3, 2, len - 3).transpose();
  }
}

void ekf_augment(VectorXf &x, MatrixXf &P, const vector<VectorXf> &zn,
                 const MatrixXf &Re) {
  for (unsigned long i = 0; i < zn.size(); i++)
    ekf_add_one_z(x, P, zn[i], Re);
}