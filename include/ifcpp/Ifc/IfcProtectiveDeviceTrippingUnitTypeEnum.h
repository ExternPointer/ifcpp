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
	// TYPE IfcProtectiveDeviceTrippingUnitTypeEnum = ENUMERATION OF	(ELECTROMAGNETIC	,ELECTRONIC	,RESIDUALCURRENT	,THERMAL	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcProtectiveDeviceTrippingUnitTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcProtectiveDeviceTrippingUnitTypeEnumEnum
		{
			ENUM_ELECTROMAGNETIC,
			ENUM_ELECTRONIC,
			ENUM_RESIDUALCURRENT,
			ENUM_THERMAL,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcProtectiveDeviceTrippingUnitTypeEnum() = default;
		IfcProtectiveDeviceTrippingUnitTypeEnum( IfcProtectiveDeviceTrippingUnitTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 2749697471; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcProtectiveDeviceTrippingUnitTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcProtectiveDeviceTrippingUnitTypeEnumEnum m_enum;
	};
}
