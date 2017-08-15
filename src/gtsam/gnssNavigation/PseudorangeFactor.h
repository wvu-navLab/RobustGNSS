/**
 *  @file   PseudorangeFactor.h
 *  @author Ryan Watson and Jason Gross
 *  @brief  Header file for Pseudorange factor
 **/

#pragma once

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Point4.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/gnssNavigation/gnssStateVec.h>

namespace gtsam {

class GTSAM_EXPORT PseudorangeFactor: public NoiseModelFactor1<gnssStateVec> {

private:
  typedef NoiseModelFactor1<gnssStateVec> Base;
  double measured_;
  gnssStateVec h_;
  Point3 satXYZ_;

public:

  typedef boost::shared_ptr<PseudorangeFactor> shared_ptr;
  typedef PseudorangeFactor This;

  PseudorangeFactor(): measured_(0) { h_=(Matrix(1,5)<<1,1,1,1,1).finished(); }

  virtual ~PseudorangeFactor() {}

  PseudorangeFactor(Key key, const double deltaObs, const Matrix obsMap,
        const Point3 satXYZ, const SharedNoiseModel& model):
    Base(model, key), measured_(deltaObs), satXYZ_(satXYZ) {h_=obsMap;}

  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new PseudorangeFactor(*this))); }

  /// vector of errors
  Vector evaluateError(const gnssStateVec& q,
      boost::optional<Matrix&> H = boost::none) const;

private:

  /// Serialization function
  friend class boost::serialization::access;
  template<class ARCHIVE>
  void serialize(ARCHIVE & ar, const unsigned int /*version*/) {
    ar
        & boost::serialization::make_nvp("NoiseModelFactor1",
            boost::serialization::base_object<Base>(*this));
    ar & BOOST_SERIALIZATION_NVP(measured_);
  }

}; // PseudorangeFactor Factor
} // namespace
