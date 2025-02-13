/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcTextAlignment = STRING;
	class IFCQUERY_EXPORT IfcTextAlignment : public BuildingObject
	{
	public:
		IfcTextAlignment() = default;
		IfcTextAlignment( std::string value );
		virtual uint32_t classID() const { return 1460886941; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcTextAlignment> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		std::string m_value;
	};
}
