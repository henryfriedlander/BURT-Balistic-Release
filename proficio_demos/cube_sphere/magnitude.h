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

#ifndef BARRETT_SYSTEMS_MAGNITUDE_H_
#define BARRETT_SYSTEMS_MAGNITUDE_H_


#include <cmath>

#include <Eigen/Core>

#include <barrett/detail/ca_macro.h>
#include <barrett/units.h>
#include <barrett/math.h>
#include <barrett/systems/abstract/system.h>
#include <barrett/systems/abstract/single_io.h>

namespace barrett {
namespace systems {


template <typename T0, typename T1>
class Magnitude : public systems::SingleIO<T0, T1> {
BARRETT_UNITS_FIXED_SIZE_TYPEDEFS;

public:
	Magnitude(const std::string& sysName = "modXYZ") :
	systems::SingleIO<T0, T1>(sysName)
	{}

protected:
	virtual void operate() {
		dir = this->input.getValue();
		depth = dir.norm();
		this->outputValue->setData(&depth);
	}

	cf_type dir;
	double depth;

private:
	DISALLOW_COPY_AND_ASSIGN(Magnitude);

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


}
}

#endif /* BARRETT_SYSTEMS_NORMALIZE_H_ */

