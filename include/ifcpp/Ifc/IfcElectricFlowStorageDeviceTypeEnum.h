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
	// TYPE IfcElectricFlowStorageDeviceTypeEnum = ENUMERATION OF	(BATTERY	,CAPACITOR	,CAPACITORBANK	,COMPENSATOR	,HARMONICFILTER	,INDUCTOR	,INDUCTORBANK	,RECHARGER	,UPS	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcElectricFlowStorageDeviceTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcElectricFlowStorageDeviceTypeEnumEnum
		{
			ENUM_BATTERY,
			ENUM_CAPACITOR,
			ENUM_CAPACITORBANK,
			ENUM_COMPENSATOR,
			ENUM_HARMONICFILTER,
			ENUM_INDUCTOR,
			ENUM_INDUCTORBANK,
			ENUM_RECHARGER,
			ENUM_UPS,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcElectricFlowStorageDeviceTypeEnum() = default;
		IfcElectricFlowStorageDeviceTypeEnum( IfcElectricFlowStorageDeviceTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3044747827; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcElectricFlowStorageDeviceTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcElectricFlowStorageDeviceTypeEnumEnum m_enum;
	};
}
