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
	// TYPE IfcLampTypeEnum = ENUMERATION OF	(COMPACTFLUORESCENT	,FLUORESCENT	,HALOGEN	,HIGHPRESSUREMERCURY	,HIGHPRESSURESODIUM	,LED	,METALHALIDE	,OLED	,TUNGSTENFILAMENT	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcLampTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcLampTypeEnumEnum
		{
			ENUM_COMPACTFLUORESCENT,
			ENUM_FLUORESCENT,
			ENUM_HALOGEN,
			ENUM_HIGHPRESSUREMERCURY,
			ENUM_HIGHPRESSURESODIUM,
			ENUM_LED,
			ENUM_METALHALIDE,
			ENUM_OLED,
			ENUM_TUNGSTENFILAMENT,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcLampTypeEnum() = default;
		IfcLampTypeEnum( IfcLampTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 185388416; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcLampTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcLampTypeEnumEnum m_enum;
	};
}
