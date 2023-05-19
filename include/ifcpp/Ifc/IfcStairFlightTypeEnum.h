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
	// TYPE IfcStairFlightTypeEnum = ENUMERATION OF	(CURVED	,FREEFORM	,SPIRAL	,STRAIGHT	,WINDER	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcStairFlightTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcStairFlightTypeEnumEnum
		{
			ENUM_CURVED,
			ENUM_FREEFORM,
			ENUM_SPIRAL,
			ENUM_STRAIGHT,
			ENUM_WINDER,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcStairFlightTypeEnum() = default;
		IfcStairFlightTypeEnum( IfcStairFlightTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3038022802; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcStairFlightTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcStairFlightTypeEnumEnum m_enum;
	};
}
