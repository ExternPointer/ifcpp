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

namespace IFC4X3
{
	// TYPE IfcText = STRING;
	class IFCQUERY_EXPORT IfcText : public IfcSimpleValue
	{
	public:
		IfcText() = default;
		IfcText( std::string value );
		virtual uint32_t classID() const { return 2801250643; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcText> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		std::string m_value;
	};
}
