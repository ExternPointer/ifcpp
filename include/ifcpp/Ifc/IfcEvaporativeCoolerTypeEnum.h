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
	// TYPE IfcEvaporativeCoolerTypeEnum = ENUMERATION OF	(DIRECTEVAPORATIVEAIRWASHER	,DIRECTEVAPORATIVEPACKAGEDROTARYAIRCOOLER	,DIRECTEVAPORATIVERANDOMMEDIAAIRCOOLER	,DIRECTEVAPORATIVERIGIDMEDIAAIRCOOLER	,DIRECTEVAPORATIVESLINGERSPACKAGEDAIRCOOLER	,INDIRECTDIRECTCOMBINATION	,INDIRECTEVAPORATIVECOOLINGTOWERORCOILCOOLER	,INDIRECTEVAPORATIVEPACKAGEAIRCOOLER	,INDIRECTEVAPORATIVEWETCOIL	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcEvaporativeCoolerTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcEvaporativeCoolerTypeEnumEnum
		{
			ENUM_DIRECTEVAPORATIVEAIRWASHER,
			ENUM_DIRECTEVAPORATIVEPACKAGEDROTARYAIRCOOLER,
			ENUM_DIRECTEVAPORATIVERANDOMMEDIAAIRCOOLER,
			ENUM_DIRECTEVAPORATIVERIGIDMEDIAAIRCOOLER,
			ENUM_DIRECTEVAPORATIVESLINGERSPACKAGEDAIRCOOLER,
			ENUM_INDIRECTDIRECTCOMBINATION,
			ENUM_INDIRECTEVAPORATIVECOOLINGTOWERORCOILCOOLER,
			ENUM_INDIRECTEVAPORATIVEPACKAGEAIRCOOLER,
			ENUM_INDIRECTEVAPORATIVEWETCOIL,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcEvaporativeCoolerTypeEnum() = default;
		IfcEvaporativeCoolerTypeEnum( IfcEvaporativeCoolerTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1002142388; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcEvaporativeCoolerTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcEvaporativeCoolerTypeEnumEnum m_enum;
	};
}
