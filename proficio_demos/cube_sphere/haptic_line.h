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

#ifndef BARRETT_SYSTEMS_HAPTIC_LINE_H_
#define BARRETT_SYSTEMS_HAPTIC_LINE_H_


#include <cmath>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class HapticLine : public HapticObject {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	HapticLine(const cp_type& center, const int& xOryOrzIsChanging,
			const std::string& sysName = "HapticLine") :
		HapticObject(sysName),
		c(center), xyz(xOryOrzIsChanging),
		depth(0.0), dir(0.0)
	{}
	virtual ~HapticLine() { mandatoryCleanUp(); }

	const cp_type& getCenter() const { return c; }

protected:
	virtual void operate() {
		pos = c - input.getValue();
		pos[xyz] = 0;
		
		depth = pos.norm();
		/*
		if (depth > 0.4){
			depth = 0.4;
		}*/
		//printf("depth: %6.3f\n", depth);
		dir = pos; // / depth;
		/*
		 * find vector from input point to point on the line
		 * Get distance from input to point on line with the same (wlog) x value
		 * then get the 2d distance between the y,z of the input to the y,z of the line
		 * 
		 */
		//printf("line dir: [%6.3f, %6.3f, %6.3f]\n", dir[0], dir[1], dir[2]);
		depthOutputValue->setData(&depth);
		directionOutputValue->setData(&dir);
	}

	cp_type c;
	int xyz;

	// state & temporaries
	cf_type pos;

	double depth;
	cf_type dir;

private:
	DISALLOW_COPY_AND_ASSIGN(HapticLine);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}

#endif /* BARRETT_SYSTEMS_HAPTIC_LINE_H_ */

