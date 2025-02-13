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
	// TYPE IfcDoorTypeEnum = ENUMERATION OF	(BOOM_BARRIER	,DOOR	,GATE	,TRAPDOOR	,TURNSTILE	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcDoorTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcDoorTypeEnumEnum
		{
			ENUM_BOOM_BARRIER,
			ENUM_DOOR,
			ENUM_GATE,
			ENUM_TRAPDOOR,
			ENUM_TURNSTILE,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcDoorTypeEnum() = default;
		IfcDoorTypeEnum( IfcDoorTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1970628803; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcDoorTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcDoorTypeEnumEnum m_enum;
	};
}
