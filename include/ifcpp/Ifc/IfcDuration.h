/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcSimpleValue.h"
#include "IfcTimeOrRatioSelect.h"

namespace IFC4X3
{
	// TYPE IfcDuration = STRING;
	class IFCQUERY_EXPORT IfcDuration : public IfcSimpleValue, public IfcTimeOrRatioSelect
	{
	public:
		IfcDuration() = default;
		IfcDuration( std::string value );
		virtual uint32_t classID() const { return 2541165894; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcDuration> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		std::string m_value;
	};
}
