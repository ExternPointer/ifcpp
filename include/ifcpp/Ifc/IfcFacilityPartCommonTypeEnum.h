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
	// TYPE IfcFacilityPartCommonTypeEnum = ENUMERATION OF	(ABOVEGROUND	,BELOWGROUND	,JUNCTION	,LEVELCROSSING	,SEGMENT	,SUBSTRUCTURE	,SUPERSTRUCTURE	,TERMINAL	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcFacilityPartCommonTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcFacilityPartCommonTypeEnumEnum
		{
			ENUM_ABOVEGROUND,
			ENUM_BELOWGROUND,
			ENUM_JUNCTION,
			ENUM_LEVELCROSSING,
			ENUM_SEGMENT,
			ENUM_SUBSTRUCTURE,
			ENUM_SUPERSTRUCTURE,
			ENUM_TERMINAL,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcFacilityPartCommonTypeEnum() = default;
		IfcFacilityPartCommonTypeEnum( IfcFacilityPartCommonTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1019252178; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcFacilityPartCommonTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcFacilityPartCommonTypeEnumEnum m_enum;
	};
}
