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
	// TYPE IfcRadioActivityMeasure = REAL;
	class IFCQUERY_EXPORT IfcRadioActivityMeasure : public IfcDerivedMeasureValue
	{
	public:
		IfcRadioActivityMeasure() = default;
		IfcRadioActivityMeasure( double value );
		virtual uint32_t classID() const { return 3972513137; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcRadioActivityMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		double m_value;
	};
}
