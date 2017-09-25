/**
 *  @file   PseudorangeFactor.cpp
 *  @author Ryan Watson & Jason Gross
 *  @brief  Implementation file for pseudorange factor
 **/

#include "PseudorangeFactor.h"

using namespace std;

namespace gtsam {

//***************************************************************************
  Vector PseudorangeFactor::evaluateError(const gnssStateVec& q,
                                   boost::optional<Matrix&> H) const {
    if (H) { (*H) = (Matrix(1,5) << h_ ).finished(); }
    double est = (h_.transpose() * q);
    return (Vector(1) << est-measured_).finished();
  }
} // namespace
