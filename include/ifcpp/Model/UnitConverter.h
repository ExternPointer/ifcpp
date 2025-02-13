/* -*-c++-*- IfcQuery www.ifcquery.com
*
MIT License

Copyright (c) 2017 Fabian Gerold

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include <map>
#include "BasicTypes.h"
#include "StatusCallback.h"
#include "BuildingException.h"
#include "ifcpp/Ifc/IfcProject.h"
#include "ifcpp/Ifc/IfcSIPrefix.h"
#include "ifcpp/Ifc/IfcSIUnitName.h"

//\brief class to convert values from different units into meter and radian
class IFCQUERY_EXPORT UnitConverter : public StatusCallback
{
public:
	enum AngularUnit { UNDEFINED, RADIANT, DEGREE, GON, CONVERSION_BASED };
	UnitConverter();
	~UnitConverter() override = default;
	void setIfcProject( shared_ptr<IFC4X3::IfcProject> project);
	void setLengthInMeterFactor( double factor )
	{
		m_length_unit_factor = factor;
		m_length_unit_found = true;
	}
	double getLengthInMeterFactor()
	{
		if( !m_length_unit_found )
		{
                    SendLogMessage( "No length unit definition found in model", StatusCallback::Warning, __FUNC__ );
		}

		return m_length_unit_factor * m_custom_length_factor;
	}

	double getCustomLengthFactor()
	{
		return m_custom_length_factor;
	}
	void setCustomLengthFactor( double custom_factor )
	{
		m_custom_length_factor = custom_factor;
	}

	void setAngleUnit(AngularUnit unit);
	double getAngleInRadiantFactor()
	{
		if( m_angular_unit == UNDEFINED )
		{
                        SendLogMessage( "No plane angle unit definition found in model", StatusCallback::Warning, __FUNC__ );
		}
		return m_plane_angle_factor;
	}
	AngularUnit getAngularUnit() { return m_angular_unit; }
	shared_ptr<IFC4X3::IfcSIPrefix>& getLoadedPrefix() { return m_loaded_prefix; }

	void resetUnitFactors();
	void resetComplete();

protected:
	std::map<int, double> m_prefix_map;
	shared_ptr<IFC4X3::IfcSIPrefix>	m_loaded_prefix;
	double m_length_unit_factor = 1.0;
	double m_custom_length_factor = 1.0;
	bool m_length_unit_found = false;
	double m_plane_angle_factor = 1.0;
	AngularUnit	m_angular_unit;
};
