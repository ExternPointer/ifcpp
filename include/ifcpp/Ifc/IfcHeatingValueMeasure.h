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
	// TYPE IfcHeatingValueMeasure = REAL;
	class IFCQUERY_EXPORT IfcHeatingValueMeasure : public IfcDerivedMeasureValue
	{
	public:
		IfcHeatingValueMeasure() = default;
		IfcHeatingValueMeasure( double value );
		virtual uint32_t classID() const { return 1158859006; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcHeatingValueMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		double m_value;
	};
}
