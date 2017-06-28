/*
	Copyright 2009, 2010, 2011, 2012 Barrett Technology <support@barrett.com>

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
 * haptic_ball.h
 *
 *  Created on: Feb 19, 2010
 *      Author: dc
 */

#ifndef BARRETT_SYSTEMS_VECTOR_SEPARATOR_H_
#define BARRETT_SYSTEMS_VECTOR_SEPARATOR_H_


#include <cmath>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/haptic_object.h>


namespace barrett {
namespace systems {


class VectorSeparator : public HapticObject {
	BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	VectorSeparator(const std::string& sysName = "HapticBall") :
		HapticObject(sysName) {}
	virtual ~VectorSeparator() { mandatoryCleanUp(); }

protected:
	virtual void operate() {
		input = input.getValue();
		depth = input.norm();
		dir = input / depth;
		
		depthOutputValue->setData(&depth);
		directionOutputValue->setData(&dir);
	}

	double depth;
	cf_type input;
	cf_type dir;

private:
	DISALLOW_COPY_AND_ASSIGN(VectorSeparator);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}


#endif /* BARRETT_SYSTEMS_Vector_Separator_H_ */

