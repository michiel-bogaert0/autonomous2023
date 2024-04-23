// Copyright:
//   * 2018 Christoph Dalitz and Jens Wilberg
//     Niederrhein University of Applied Sciences,
//     Institute for Pattern Recognition,
//     Reinarzstr. 49, 47805 Krefeld, Germany
//     <http://www.hsnr.de/ipattern/>
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "kdtree.hpp"
#include <algorithm>
#include <limits>
#include <math.h>
#include <stdexcept>

namespace Kdtree {

//--------------------------------------------------------------
// function object for comparing only dimension d of two vecotrs
//--------------------------------------------------------------
class compare_dimension {
public:
  explicit compare_dimension(size_t dim) { d = dim; }
  bool operator()(const KdNode &p, const KdNode &q) {
    return (p.point[d] < q.point[d]);
  }
  size_t d;
};

//--------------------------------------------------------------
// internal node structure used by kdtree
//--------------------------------------------------------------
class kdtree_node {
public:
  kdtree_node() {
    dataindex = cutdim = 0;
    loson = hison = (kdtree_node *)NULL;
  }
  ~kdtree_node() {
    if (loson)
      delete loson;
    if (hison)
      delete hison;
  }
  // index of node data in kdtree array "allnodes"
  size_t dataindex;
  // cutting dimension
  size_t cutdim;
  // value of point
  // double cutval; // == point[cutdim]
  CoordPoint point;
  //  roots of the two subtrees
  kdtree_node *loson, *hison;
  // bounding rectangle of this node's subtree
  CoordPoint lobound, upbound;
};

//--------------------------------------------------------------
// different distance metrics
//--------------------------------------------------------------
class DistanceMeasure {
public:
  DistanceMeasure() {}
  virtual ~DistanceMeasure() {}
  virtual double distance(const CoordPoint &p, const CoordPoint &q) = 0;
  virtual double coordinate_distance(double x, double y, size_t dim) = 0;
};
// Maximum distance (Linfinite norm)
class DistanceL0 : virtual public DistanceMeasure {
  DoubleVector *w;

public:
  DistanceL0(const DistanceL0 &other) { w = new DoubleVector(*(other.w)); }
  DistanceL0(const DoubleVector *weights = NULL) {
    if (weights)
      w = new DoubleVector(*weights);
    else
      w = (DoubleVector *)NULL;
  }
  ~DistanceL0() {
    if (w)
      delete w;
  }
  DistanceL0 &operator=(const DistanceL0 &other) {
    if (this != &other) {
      delete w; // Deallocate memory for the existing DoubleVector
      w = new DoubleVector(
          *(other.w)); // Allocate memory for a new DoubleVector and copy the
                       // data Copy other members...
    }
    return *this;
  }
  double distance(const CoordPoint &p, const CoordPoint &q) override {
    size_t i;
    double dist, test;
    if (w) {
      dist = (*w)[0] * fabs(p[0] - q[0]);
      for (i = 1; i < p.size(); i++) {
        test = (*w)[i] * fabs(p[i] - q[i]);
        if (test > dist)
          dist = test;
      }
    } else {
      dist = fabs(p[0] - q[0]);
      for (i = 1; i < p.size(); i++) {
        test = fabs(p[i] - q[i]);
        if (test > dist)
          dist = test;
      }
    }
    return dist;
  }
  double coordinate_distance(double x, double y, size_t dim) override {
    if (w)
      return (*w)[dim] * fabs(x - y);
    else
      return fabs(x - y);
  }
};
// Manhatten distance (L1 norm)
class DistanceL1 : virtual public DistanceMeasure {
  DoubleVector *w;

public:
  DistanceL1(const DistanceL1 &other) { w = new DoubleVector(*(other.w)); }
  DistanceL1(const DoubleVector *weights = NULL) {
    if (weights)
      w = new DoubleVector(*weights);
    else
      w = (DoubleVector *)NULL;
  }
  ~DistanceL1() {
    if (w)
      delete w;
  }
  DistanceL1 &operator=(const DistanceL1 &other) {
    if (this != &other) {
      delete w; // Deallocate memory for the existing DoubleVector
      w = new DoubleVector(
          *(other.w)); // Allocate memory for a new DoubleVector and copy the
                       // data Copy other members...
    }
    return *this;
  }
  double distance(const CoordPoint &p, const CoordPoint &q) override {
    size_t i;
    double dist = 0.0;
    if (w) {
      for (i = 0; i < p.size(); i++)
        dist += (*w)[i] * fabs(p[i] - q[i]);
    } else {
      for (i = 0; i < p.size(); i++)
        dist += fabs(p[i] - q[i]);
    }
    return dist;
  }
  double coordinate_distance(double x, double y, size_t dim) override {
    if (w)
      return (*w)[dim] * fabs(x - y);
    else
      return fabs(x - y);
  }
};
// Euklidean distance (L2 norm) (squared)
class DistanceL2 : virtual public DistanceMeasure {
  DoubleVector *w;

public:
  DistanceL2(const DistanceL2 &other) { w = new DoubleVector(*(other.w)); }
  DistanceL2(const DoubleVector *weights = NULL) {
    if (weights)
      w = new DoubleVector(*weights);
    else
      w = (DoubleVector *)NULL;
  }
  ~DistanceL2() {
    if (w)
      delete w;
  }
  DistanceL2 &operator=(const DistanceL2 &other) {
    if (this != &other) {
      delete w; // Deallocate memory for the existing DoubleVector
      w = new DoubleVector(
          *(other.w)); // Allocate memory for a new DoubleVector and copy the
                       // data Copy other members...
    }
    return *this;
  }
  double distance(const CoordPoint &p, const CoordPoint &q) override {
    size_t i;
    double dist = 0.0;
    if (w) {
      for (i = 0; i < p.size(); i++)
        dist += (*w)[i] * (p[i] - q[i]) * (p[i] - q[i]);
    } else {
      for (i = 0; i < p.size(); i++)
        dist += (p[i] - q[i]) * (p[i] - q[i]);
    }
    return dist;
  }
  double coordinate_distance(double x, double y, size_t dim) override {
    if (w)
      return (*w)[dim] * (x - y) * (x - y);
    else
      return (x - y) * (x - y);
  }
};

//--------------------------------------------------------------
// destructor and constructor of kdtree
//--------------------------------------------------------------
KdTree::~KdTree() {
  if (root)
    delete root;
  delete distance;
}
// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
KdTree::KdTree(const KdNodeVector *nodes, int distance_type /*=2*/) {
  size_t i, j;
  double val;
  // copy over input data
  if (!nodes || nodes->empty())
    throw std::invalid_argument(
        "kdtree::KdTree(): argument nodes must not be empty");
  dimension = nodes->begin()->point.size();
  allnodes = *nodes;
  // initialize distance values
  distance = NULL;
  this->distance_type = -1;
  set_distance(distance_type);
  // compute global bounding box
  lobound = nodes->begin()->point;
  upbound = nodes->begin()->point;
  for (i = 1; i < nodes->size(); i++) {
    for (j = 0; j < dimension; j++) {
      val = allnodes[i].point[j];
      if (lobound[j] > val)
        lobound[j] = val;
      if (upbound[j] < val)
        upbound[j] = val;
    }
  }
  // build tree recursively
  root = build_tree(0, 0, allnodes.size());
}

// distance_type can be 0 (Maximum), 1 (Manhatten), or 2 (Euklidean [squared])
void KdTree::set_distance(int distance_type,
                          const DoubleVector *weights /*=NULL*/) {
  if (distance)
    delete distance;
  this->distance_type = distance_type;
  if (distance_type == 0) {
    distance = static_cast<DistanceMeasure *>(new DistanceL0(weights));
  } else if (distance_type == 1) {
    distance = static_cast<DistanceMeasure *>(new DistanceL1(weights));
  } else {
    distance = static_cast<DistanceMeasure *>(new DistanceL2(weights));
  }
}

//--------------------------------------------------------------
// recursive build of tree
// "a" and "b"-1 are the lower and upper indices
// from "allnodes" from which the subtree is to be built
//--------------------------------------------------------------
kdtree_node *KdTree::build_tree(size_t depth, size_t a, size_t b) {
  kdtree_node *node = new kdtree_node();
  node->lobound = lobound;
  node->upbound = upbound;
  node->cutdim = depth % dimension;
  if (b - a <= 1) {
    node->dataindex = a;
    node->point = allnodes[a].point;
  } else {
    size_t m;
    double temp, cutval;
    m = (a + b) / 2;
    std::nth_element(allnodes.begin() + a, allnodes.begin() + m,
                     allnodes.begin() + b, compare_dimension(node->cutdim));
    node->point = allnodes[m].point;
    cutval = allnodes[m].point[node->cutdim];
    node->dataindex = m;
    if (m - a > 0) {
      temp = upbound[node->cutdim];
      upbound[node->cutdim] = cutval;
      node->loson = build_tree(depth + 1, a, m);
      upbound[node->cutdim] = temp;
    }
    if (b - m > 1) {
      temp = lobound[node->cutdim];
      lobound[node->cutdim] = cutval;
      node->hison = build_tree(depth + 1, m + 1, b);
      lobound[node->cutdim] = temp;
    }
  }
  return node;
}

//--------------------------------------------------------------
// k nearest neighbor search
// returns the *k* nearest neighbors of *point* in O(log(n))
// time. The result is returned in *result* and is sorted by
// distance from *point*.
// The optional search predicate is a callable class (aka "functor")
// derived from KdNodePredicate. When Null (default, no search
// predicate is applied).
//--------------------------------------------------------------
void KdTree::k_nearest_neighbors(const CoordPoint &point, size_t k,
                                 KdNodeVector *result,
                                 KdNodePredicate *pred /*=NULL*/) {
  size_t i;
  KdNode temp;
  searchpredicate = pred;

  result->clear();
  if (k < 1)
    return;
  if (point.size() != dimension)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");

  // collect result of k values in neighborheap
  // std::priority_queue<nn4heap, std::vector<nn4heap>, compare_nn4heap>*
  // neighborheap = new std::priority_queue<nn4heap, std::vector<nn4heap>,
  // compare_nn4heap>();
  SearchQueue *neighborheap = new SearchQueue();
  if (k > allnodes.size()) {
    // when more neighbors asked than nodes in tree, return everything
    k = allnodes.size();
    for (i = 0; i < k; i++) {
      if (!(searchpredicate && !(*searchpredicate)(allnodes[i])))
        neighborheap->push(
            nn4heap(i, distance->distance(allnodes[i].point, point)));
    }
  } else {
    neighbor_search(point, root, k, neighborheap);
  }

  // copy over result sorted by distance
  // (we must revert the vector for ascending order)
  while (!neighborheap->empty()) {
    i = neighborheap->top().dataindex;
    neighborheap->pop();
    result->push_back(allnodes[i]);
  }
  // beware that less than k results might have been returned
  k = result->size();
  for (i = 0; i < k / 2; i++) {
    temp = (*result)[i];
    (*result)[i] = (*result)[k - 1 - i];
    (*result)[k - 1 - i] = temp;
  }
  delete neighborheap;
}

//--------------------------------------------------------------
// range nearest neighbor search
// returns the nearest neighbors of *point* in the given range
// *r*. The result is returned in *result* and is sorted by
// distance from *point*.
//--------------------------------------------------------------
void KdTree::range_nearest_neighbors(const CoordPoint &point, double r,
                                     KdNodeVector *result) {
  KdNode temp;

  result->clear();
  if (point.size() != dimension)
    throw std::invalid_argument(
        "kdtree::k_nearest_neighbors(): point must be of same dimension as "
        "kdtree");
  if (this->distance_type == 2) {
    // if euclidien distance is used the range must be squared because we
    // get squared distances from this implementation
    r *= r;
  }

  // collect result in range_result
  std::vector<size_t> range_result;
  range_search(point, root, r, &range_result);

  // copy over result
  for (std::vector<size_t>::iterator i = range_result.begin();
       i != range_result.end(); ++i) {
    result->push_back(allnodes[*i]);
  }

  // clear vector
  range_result.clear();
}

//--------------------------------------------------------------
// recursive function for nearest neighbor search in subtree
// under *node*. Stores result in *neighborheap*.
// returns "true" when no nearer neighbor elsewhere possible
//--------------------------------------------------------------
bool KdTree::neighbor_search(const CoordPoint &point, kdtree_node *node,
                             size_t k, SearchQueue *neighborheap) {
  double curdist, dist;

  curdist = distance->distance(point, node->point);
  if (!(searchpredicate && !(*searchpredicate)(allnodes[node->dataindex]))) {
    if (neighborheap->size() < k) {
      neighborheap->push(nn4heap(node->dataindex, curdist));
    } else if (curdist < neighborheap->top().distance) {
      neighborheap->pop();
      neighborheap->push(nn4heap(node->dataindex, curdist));
    }
  }
  // first search on side closer to point
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->loson)
      if (neighbor_search(point, node->loson, k, neighborheap))
        return true;
  } else {
    if (node->hison)
      if (neighbor_search(point, node->hison, k, neighborheap))
        return true;
  }
  // second search on farther side, if necessary
  if (neighborheap->size() < k) {
    dist = std::numeric_limits<double>::max();
  } else {
    dist = neighborheap->top().distance;
  }
  if (point[node->cutdim] < node->point[node->cutdim]) {
    if (node->hison && bounds_overlap_ball(point, dist, node->hison))
      if (neighbor_search(point, node->hison, k, neighborheap))
        return true;
  } else {
    if (node->loson && bounds_overlap_ball(point, dist, node->loson))
      if (neighbor_search(point, node->loson, k, neighborheap))
        return true;
  }

  if (neighborheap->size() == k)
    dist = neighborheap->top().distance;
  return ball_within_bounds(point, dist, node);
}

//--------------------------------------------------------------
// recursive function for range search in subtree under *node*.
// Stores result in *range_result*.
//--------------------------------------------------------------
void KdTree::range_search(const CoordPoint &point, kdtree_node *node, double r,
                          std::vector<size_t> *range_result) {
  double curdist = distance->distance(point, node->point);
  if (curdist <= r) {
    range_result->push_back(node->dataindex);
  }
  if (node->loson != NULL && this->bounds_overlap_ball(point, r, node->loson)) {
    range_search(point, node->loson, r, range_result);
  }
  if (node->hison != NULL && this->bounds_overlap_ball(point, r, node->hison)) {
    range_search(point, node->hison, r, range_result);
  }
}

// returns true when the bounds of *node* overlap with the
// ball with radius *dist* around *point*
bool KdTree::bounds_overlap_ball(const CoordPoint &point, double dist,
                                 kdtree_node *node) {
  if (distance_type != 0) {
    double distsum = 0.0;
    size_t i;
    for (i = 0; i < dimension; i++) {
      if (point[i] < node->lobound[i]) { // lower than low boundary
        distsum += distance->coordinate_distance(point[i], node->lobound[i], i);
        if (distsum > dist)
          return false;
      } else if (point[i] > node->upbound[i]) { // higher than high boundary
        distsum += distance->coordinate_distance(point[i], node->upbound[i], i);
        if (distsum > dist)
          return false;
      }
    }
    return true;
  } else { // maximum distance needs different treatment
    double max_dist = 0.0;
    double curr_dist = 0.0;
    size_t i;
    for (i = 0; i < dimension; i++) {
      if (point[i] < node->lobound[i]) { // lower than low boundary
        curr_dist =
            distance->coordinate_distance(point[i], node->lobound[i], i);
      } else if (point[i] > node->upbound[i]) { // higher than high boundary
        curr_dist =
            distance->coordinate_distance(point[i], node->upbound[i], i);
      }
      if (curr_dist > max_dist) {
        max_dist = curr_dist;
      }
      if (max_dist > dist)
        return false;
    }
    return true;
  }
}

// returns true when the bounds of *node* completely contain the
// ball with radius *dist* around *point*
bool KdTree::ball_within_bounds(const CoordPoint &point, double dist,
                                kdtree_node *node) {
  size_t i;
  for (i = 0; i < dimension; i++)
    if (distance->coordinate_distance(point[i], node->lobound[i], i) <= dist ||
        distance->coordinate_distance(point[i], node->upbound[i], i) <= dist)
      return false;
  return true;
}

} // namespace Kdtree