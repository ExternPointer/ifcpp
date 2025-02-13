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
	// TYPE IfcJunctionBoxTypeEnum = ENUMERATION OF	(DATA	,POWER	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcJunctionBoxTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcJunctionBoxTypeEnumEnum
		{
			ENUM_DATA,
			ENUM_POWER,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcJunctionBoxTypeEnum() = default;
		IfcJunctionBoxTypeEnum( IfcJunctionBoxTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1844818999; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcJunctionBoxTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcJunctionBoxTypeEnumEnum m_enum;
	};
}
