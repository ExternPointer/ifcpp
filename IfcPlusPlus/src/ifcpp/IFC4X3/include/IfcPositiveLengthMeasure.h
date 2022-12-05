/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"
#include "IfcLengthMeasure.h"
#include "IfcHatchLineDistanceSelect.h"

namespace IFC4X3
{
	// TYPE IfcPositiveLengthMeasure = IfcLengthMeasure;
	class IFCQUERY_EXPORT IfcPositiveLengthMeasure : public IfcLengthMeasure, public IfcHatchLineDistanceSelect
	{
	public:
		IfcPositiveLengthMeasure() = default;
		IfcPositiveLengthMeasure( double value ) { m_value = value; }
		virtual uint32_t classID() const { return 2815919920; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPositiveLengthMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
	};
}

