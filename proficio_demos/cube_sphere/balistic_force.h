
/*
	Copyright 2009, 2010 Barrett Technology <support@barrett.com>

	This file is part of libbarrett.

	This version of libbarrett is free software: you can redistribute it
	and/or modify it under the terms of the GNU General Public License as
	published by the Free Software Foundation, either version 3 of the
	License, or (at your option) any later version.

	This version of libbarrett is distributed in the hope that it will be
	useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License along
	with this version of libbarrett.  If not, see
	<http://www.gnu.org/licenses/>.

	Further, non-binding information about licensing is available at:
	<http://wiki.barrett.com/libbarrett/wiki/LicenseNotes>
*/

/*
 * haptic_box.h
 *
 *  Created on: Apr 16, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_BALISTIC_FORCE_H_
#define BARRETT_SYSTEMS_BALISTIC_FORCE_H_


#include <cmath>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class BalisticForce : public HapticObject {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	BalisticForce(const cp_type& center, const int& xOryOrzIsChanging, const double& forceThreshold,
			const std::string& sysName = "BalisticForce") :
		HapticObject(sysName),
		c(center), xyz(xOryOrzIsChanging), ft(forceThreshold),
		thresholdMet(false), depth(0.0), dir(0.0) 
	{}
	virtual ~BalisticForce() { mandatoryCleanUp(); }

	const cp_type& getCenter() const { return c; }

protected:
	virtual void operate() {
		for (int i=0;i<3;i++){
			dir[i] = 0.0;
		}
		if (!thresholdMet){
			cforce = c - input.getValue();
			if ((barrett::math::sign(ft)==1 && cforce[xyz] >= ft) || (barrett::math::sign(ft)==-1 && cforce[xyz] <= ft)){
				thresholdMet = true;
			}
			
			depth = cforce.norm();
			dir[xyz] = cforce[xyz]; // / depth;
		}else{
			depth = 0.0;
		}
		depthOutputValue->setData(&depth);
		directionOutputValue->setData(&dir);
	}

	cp_type c;
	int xyz;
	double ft;

	// state & temporaries
	cf_type cforce;

	bool thresholdMet;
	double depth;
	cf_type dir;
	

private:
	DISALLOW_COPY_AND_ASSIGN(BalisticForce);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}

#endif /* BARRETT_SYSTEMS_HAPTIC_LINE_H_ */

