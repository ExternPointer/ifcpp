/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcDerivedMeasureValue.h"

namespace IFC4X3
{
	// TYPE IfcRotationalFrequencyMeasure = REAL;
	class IFCQUERY_EXPORT IfcRotationalFrequencyMeasure : public IfcDerivedMeasureValue
	{
	public:
		IfcRotationalFrequencyMeasure() = default;
		IfcRotationalFrequencyMeasure( double value );
		virtual uint32_t classID() const { return 2133746277; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcRotationalFrequencyMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		double m_value;
	};
}
