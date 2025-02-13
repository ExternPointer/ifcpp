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
	// TYPE IfcObjectTypeEnum = ENUMERATION OF	(ACTOR	,CONTROL	,GROUP	,PROCESS	,PRODUCT	,PROJECT	,RESOURCE	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcObjectTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcObjectTypeEnumEnum
		{
			ENUM_ACTOR,
			ENUM_CONTROL,
			ENUM_GROUP,
			ENUM_PROCESS,
			ENUM_PRODUCT,
			ENUM_PROJECT,
			ENUM_RESOURCE,
			ENUM_NOTDEFINED
		};

		IfcObjectTypeEnum() = default;
		IfcObjectTypeEnum( IfcObjectTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1545711075; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcObjectTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcObjectTypeEnumEnum m_enum;
	};
}
