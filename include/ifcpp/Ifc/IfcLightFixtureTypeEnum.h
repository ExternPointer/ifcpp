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
	// TYPE IfcLightFixtureTypeEnum = ENUMERATION OF	(DIRECTIONSOURCE	,POINTSOURCE	,SECURITYLIGHTING	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcLightFixtureTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcLightFixtureTypeEnumEnum
		{
			ENUM_DIRECTIONSOURCE,
			ENUM_POINTSOURCE,
			ENUM_SECURITYLIGHTING,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcLightFixtureTypeEnum() = default;
		IfcLightFixtureTypeEnum( IfcLightFixtureTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 4215032627; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcLightFixtureTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcLightFixtureTypeEnumEnum m_enum;
	};
}
