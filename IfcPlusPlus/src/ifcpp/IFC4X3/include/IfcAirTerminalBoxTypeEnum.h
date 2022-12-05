/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcAirTerminalBoxTypeEnum = ENUMERATION OF	(CONSTANTFLOW	,VARIABLEFLOWPRESSUREDEPENDANT	,VARIABLEFLOWPRESSUREINDEPENDANT	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcAirTerminalBoxTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcAirTerminalBoxTypeEnumEnum
		{
			ENUM_CONSTANTFLOW,
			ENUM_VARIABLEFLOWPRESSUREDEPENDANT,
			ENUM_VARIABLEFLOWPRESSUREINDEPENDANT,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcAirTerminalBoxTypeEnum() = default;
		IfcAirTerminalBoxTypeEnum( IfcAirTerminalBoxTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1269596434; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcAirTerminalBoxTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcAirTerminalBoxTypeEnumEnum m_enum;
	};
}

