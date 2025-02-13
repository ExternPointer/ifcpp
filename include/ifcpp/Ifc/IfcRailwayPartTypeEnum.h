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
	// TYPE IfcRailwayPartTypeEnum = ENUMERATION OF	(DILATATIONSUPERSTRUCTURE	,LINESIDESTRUCTURE	,LINESIDESTRUCTUREPART	,PLAINTRACKSUPERSTRUCTURE	,SUPERSTRUCTURE	,TRACKSTRUCTURE	,TRACKSTRUCTUREPART	,TURNOUTSUPERSTRUCTURE	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcRailwayPartTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcRailwayPartTypeEnumEnum
		{
			ENUM_DILATATIONSUPERSTRUCTURE,
			ENUM_LINESIDESTRUCTURE,
			ENUM_LINESIDESTRUCTUREPART,
			ENUM_PLAINTRACKSUPERSTRUCTURE,
			ENUM_SUPERSTRUCTURE,
			ENUM_TRACKSTRUCTURE,
			ENUM_TRACKSTRUCTUREPART,
			ENUM_TURNOUTSUPERSTRUCTURE,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcRailwayPartTypeEnum() = default;
		IfcRailwayPartTypeEnum( IfcRailwayPartTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 2181869104; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcRailwayPartTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcRailwayPartTypeEnumEnum m_enum;
	};
}
