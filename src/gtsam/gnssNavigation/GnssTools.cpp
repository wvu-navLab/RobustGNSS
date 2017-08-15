/**
 * @file   GnssTools.cpp
 * @brief  Tools required to process GNSS data -- (i.e. ECEF to ENU transformation)
 * @author Ryan Watson
 */

#include <gtsam/gnssNavigation/GnssTools.h>

namespace gtsam {

	Vector obsMap(const Point3 &p1, const Point3 &p2, const int &Trop) {
		/*
		inputs ::
			p1 --> ECEF xyz coordinates of satellite [meter]
			p2 --> ECEF xyz coordinates of receiver [meter]
			Trop --> Troposphere modeling switch
		outputs ::
			H --> measurement mapping matrix
		*/
		double r = sqrt( ((p1.x()-p2.x()))*(p1.x()-p2.x()) + ((p1.y()-p2.y())*(p1.y()-p2.y())) + ((p1.z()-p2.z())*(p1.z()-p2.z())) );
		if (Trop == 1) {
			double el = calcElNed(p1);
			double mapT = tropMap(el);
			Vector5 H; H << (p1.x()-p2.x())/r, (p1.y()- p2.y())/r,
                             (p1.z()-p2.z())/r, 1.0, mapT;
			return H;
		}
		else {
			Vector5 H; H << (p1.x()-p2.x())/r, (p1.y()- p2.y())/r,
                           (p1.z()-p2.z())/r, 1.0, 0.0;
			return H;
		}
	}

	double deltaObs(const Point3 &p1, const Point3 &p2, const double &pseudorange){
		/*
		inputs ::
			p1 --> ECEF xyz coordinates of satellite [meter]
			p2 --> ECEF xyz coordinates of receiver [meter]
			pseudorange --> observed range between satellite and receiver [meter]
		outputs ::
			deltaR --> difference observed pseudorange and modeled pseudorange [meter]
		*/
		double r = norm3(p1-p2) + tropMap(calcEl(p2,p1))*tropDry(p2);
		return pseudorange - r;
	}

	Point3 xyz2llh(const Point3 &p1){
		/*
		inputs ::
			p1 --> ECEF xyz receiver coordinates [meter]
		output ::
			posLLH --> latitude, longitude, height [rad,rad,meter]
		*/
		double e = std::sqrt(1.0-(semiMinor/semiMajor)*(semiMinor/semiMajor));
		double r = std::sqrt( ( p1.x()*p1.x() ) + (p1.y()*p1.y()) );
		double F = 54*(semiMinor*semiMinor)*(p1.z()*p1.z());
		double G = r*r + ((1- (e*e))*(p1.z()*p1.z())) - ((e*e)*((semiMajor*semiMajor)-(semiMinor*semiMinor)));
		double c = ( ((e*e*e*e)*F*r*r)/(G*G*G) );
		double s = std::cbrt( ( 1 + c + std::sqrt(c*c+2*c) ) );
		double P = F /  ( 3 * ( (s+1/s+1)*(s+1/s+1) ) * (G*G) );
		double Q = sqrt( 1+(2*e*e*e*e*P) );
		double ro = -1*(P*e*e*r)/(1+Q) + std::sqrt((semiMajor*semiMajor/2) * (1+1/Q) - (P*(1-e*e)*p1.z()*p1.z())/(Q*(1+Q)) - P*(r*r)/2);
		double tmp = ( r - e*e *ro ) * (r - e*e * ro);
		double U = std::sqrt( tmp + p1.z()*p1.z() );
		double V = std::sqrt(tmp + (1 - e*e)*(p1.z()*p1.z()));
		double zo = ( (semiMinor*semiMinor * p1.z()) / (semiMajor*V) );
		double height = U*( 1 - ((semiMinor*semiMinor)/(semiMajor*V)) );
		double lat = atan( (p1.z() + ( e*(semiMajor/semiMinor) ) * ( e*(semiMajor/semiMinor) ) * zo) / r);
		double lon = atan(p1.y()/p1.x());
		if ( (p1.x() < 0) && (p1.y() >0) ) { double lon = lon + M_PI; }
		if ( (p1.x() >= 0) && (p1.y() < 0) ) { double lon = lon - M_PI; }
	 	Point3 llhPos;
		return llhPos = Point3(lat,lon,height);
}

  Point3 inertialToECEF( const Point3& inertialPosition, const double t, const double t0){
    /*
     * inputs ::
     *  inertialPos -- > ECI position vector
     *  t --> current time [sec]
     *  t0 --> time when coordinate frames aligned [sec]
     * ouputs ::
     *  ecefPos --> ECEF position vector [meters]
    */
		double cT = cos(earthRot*(t-t0));
		double sT = sin(earthRot*(t-t0));
		Matrix Rie = (Matrix(3,3) << cT, sT, 0, -sT, cT, 0, 0, 0, 1 ).finished();
		Point3 ecefPosition= Rie*inertialPosition;
		return ecefPosition;
	}

  Point3 enu2xyz(const Point3& p1, const Point3& p2) {
		/*
		inputs ::
			p1 --> enu coordinates [meter]
			p2 --> ECEF xyz origin coordinates [meter]
     outputs ::
      posXYZ ---> ECEF XYZ position vector [meters]
		*/
		Vector3 orgLLH = xyz2llh(p2);
		double sinPhi = sin(orgLLH(0));
		double cosPhi = cos(orgLLH(0));
		double sinLam = sin(orgLLH(1));
		double cosLam = cos(orgLLH(1));
		Matrix R = ( Matrix(3,3) << -1*sinLam, cosLam, 0,
                                -1*sinPhi*cosLam, -1*sinPhi*sinLam, cosPhi,
                                cosPhi*cosLam, cosPhi*sinLam, sinPhi ).finished();
		Point3 deltaXYZ;
		deltaXYZ = R.inverse()*p1;
		return p2 + deltaXYZ;
    }

    Point3 ned2enu(const Point3& p1) {
        Matrix enuConv = ( Matrix(3,3) << 0, 1, 0, 1, 0, 0, 0, 0, -1 ).finished();
        return enuConv*p1;
    }

	Point3 xyz2enu(const Point3 &p1, const Point3 &p2){
		/*
		inputs ::
			p1 --> ECEF xyz coordinates [meter]
			p2 --> ECEF xyz origin coordinates [meter]
    outputs ::
      posENU --> ENU position coordinates [meters]
		*/

		Vector3 posDiff = p1 - p2;
		Vector3 orgLLH = xyz2llh(p2);
		double sinPhi = sin(orgLLH(0));
		double cosPhi = cos(orgLLH(0));
		double sinLam = sin(orgLLH(1));
		double cosLam = cos(orgLLH(1));
		Matrix R = ( Matrix(3,3) << (-1*sinLam), cosLam, 0, ((-1*sinPhi)*cosLam), ((-1*sinPhi)*sinLam), cosPhi, (cosPhi*cosLam), (cosPhi*sinLam), sinPhi ).finished();
		Vector3 pos;
		pos = R*posDiff;
		Point3 posENU;
		return posENU = Point3(pos(0), pos(1), pos(2));
	}

  double calcElNed(const Point3& p1){
  /*
   * inputs ::
   *  p1 --> ECEF xyz location [meters]
   * outpus ::
   *  EL --> elevation angle [rad]
  */
		Vector3 posENU = ned2enu(p1);
		double El =atan2(posENU(2), posENU.norm());
		return El;
	}

	double calcEl(const Point3& p1, const Point3& p2){
		/*
		inputs ::
			p1 --> ECEF xyz satellite coordinates [meter]
			p2 ---> ECEF xyz receiver coordinates [meter]
		output ::
			El --> elevation angle [rad]
		*/
		Vector3 posENU = xyz2enu(p1,p2);
		double El = atan2(posENU(2), posENU.norm());
		return El;
	}

	double tropMap(const double& El){
		/*
		inputs ::
			El --> receiver to satellite elevation angle [rad]
		output ::
			m --> troposphere delay [meter]
		*/
		double m = 1.001/sqrt(0.002001+(sin(El)*sin(El)));
		return m;
	}

	double tropDry(const Point3& p1){
		/*
		inputs ::
			p1 --> ECEF xyz receiver coordinated [meter]
		output ::
			tropDelta --> delay associated with troposphere [meter]
		*/
		Vector3 posLLH = xyz2llh(p1);
		double recHeight = posLLH(2)/1000;   // receiver height [km]
		double pSea = stdPressure* pow( ( (stdTemp-(6.5*recHeight))/(stdTemp+6.5)), 5.2459587 );
		double tropDelta =  ( (1e-6/5) * ( (77.624*(pSea/(stdTemp+6.5))) * (40136+148.72*((stdTemp-(6.5*recHeight))-stdTemp)) ) );
		return tropDelta;
	}

	double dopplerObs(double r1, double r2){
		/*
		inputs ::
			r1 --> carrier phase obs. timestep n
			r2 --> carrier phase obs. timestep n-1
		output ::
			tdcp --> time differenced carrier phase observable
		*/
    double tdcp = r1 - r2;
		return tdcp;
	}

  double elDepWeight(const Point3& p1, const Point3& p2, double measWeight) {
		/*
		inputs ::
			p1 --> ECEF xyz satellite coordinates [meter]
			p2 ---> ECEF xyz receiver coordinates [meter]
                        measNoise ---> initial noise applied to observable [meters]
		output ::
			r --> elevation angle dep. GNSS obs. weigth [rad]
		*/
    double el = calcEl(p1,p2);
    double r = measWeight / ( sin(el) );
    return r;
  }

	Point3 satVelocity(const Point3& p1, const Point3& p2, double ts){
		/*
		inputs ::
			p1 --> sat xyz at timestep n
			p2 --> sat xyz at timestep n-1
			ts --> timestep
		outputs ::
			satVel --> sat velocity
		*/

		Point3 satVel = ( (p1 - p2)/ (2*ts) );
		return satVel;
	}

}
