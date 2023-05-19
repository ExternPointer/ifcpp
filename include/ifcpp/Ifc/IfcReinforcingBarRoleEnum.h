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
	// TYPE IfcReinforcingBarRoleEnum = ENUMERATION OF	(ANCHORING	,EDGE	,LIGATURE	,MAIN	,PUNCHING	,RING	,SHEAR	,STUD	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcReinforcingBarRoleEnum : virtual public BuildingObject
	{
	public:
		enum IfcReinforcingBarRoleEnumEnum
		{
			ENUM_ANCHORING,
			ENUM_EDGE,
			ENUM_LIGATURE,
			ENUM_MAIN,
			ENUM_PUNCHING,
			ENUM_RING,
			ENUM_SHEAR,
			ENUM_STUD,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcReinforcingBarRoleEnum() = default;
		IfcReinforcingBarRoleEnum( IfcReinforcingBarRoleEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3202202375; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcReinforcingBarRoleEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcReinforcingBarRoleEnumEnum m_enum;
	};
}
