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
	// TYPE IfcBridgeTypeEnum = ENUMERATION OF	(ARCHED	,CABLE_STAYED	,CANTILEVER	,CULVERT	,FRAMEWORK	,GIRDER	,SUSPENSION	,TRUSS	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcBridgeTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcBridgeTypeEnumEnum
		{
			ENUM_ARCHED,
			ENUM_CABLE_STAYED,
			ENUM_CANTILEVER,
			ENUM_CULVERT,
			ENUM_FRAMEWORK,
			ENUM_GIRDER,
			ENUM_SUSPENSION,
			ENUM_TRUSS,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcBridgeTypeEnum() = default;
		IfcBridgeTypeEnum( IfcBridgeTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1536983066; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcBridgeTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcBridgeTypeEnumEnum m_enum;
	};
}
