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
	// TYPE IfcUnitEnum = ENUMERATION OF	(ABSORBEDDOSEUNIT	,AMOUNTOFSUBSTANCEUNIT	,AREAUNIT	,DOSEEQUIVALENTUNIT	,ELECTRICCAPACITANCEUNIT	,ELECTRICCHARGEUNIT	,ELECTRICCONDUCTANCEUNIT	,ELECTRICCURRENTUNIT	,ELECTRICRESISTANCEUNIT	,ELECTRICVOLTAGEUNIT	,ENERGYUNIT	,FORCEUNIT	,FREQUENCYUNIT	,ILLUMINANCEUNIT	,INDUCTANCEUNIT	,LENGTHUNIT	,LUMINOUSFLUXUNIT	,LUMINOUSINTENSITYUNIT	,MAGNETICFLUXDENSITYUNIT	,MAGNETICFLUXUNIT	,MASSUNIT	,PLANEANGLEUNIT	,POWERUNIT	,PRESSUREUNIT	,RADIOACTIVITYUNIT	,SOLIDANGLEUNIT	,THERMODYNAMICTEMPERATUREUNIT	,TIMEUNIT	,VOLUMEUNIT	,USERDEFINED);
	class IFCQUERY_EXPORT IfcUnitEnum : virtual public BuildingObject
	{
	public:
		enum IfcUnitEnumEnum
		{
			ENUM_ABSORBEDDOSEUNIT,
			ENUM_AMOUNTOFSUBSTANCEUNIT,
			ENUM_AREAUNIT,
			ENUM_DOSEEQUIVALENTUNIT,
			ENUM_ELECTRICCAPACITANCEUNIT,
			ENUM_ELECTRICCHARGEUNIT,
			ENUM_ELECTRICCONDUCTANCEUNIT,
			ENUM_ELECTRICCURRENTUNIT,
			ENUM_ELECTRICRESISTANCEUNIT,
			ENUM_ELECTRICVOLTAGEUNIT,
			ENUM_ENERGYUNIT,
			ENUM_FORCEUNIT,
			ENUM_FREQUENCYUNIT,
			ENUM_ILLUMINANCEUNIT,
			ENUM_INDUCTANCEUNIT,
			ENUM_LENGTHUNIT,
			ENUM_LUMINOUSFLUXUNIT,
			ENUM_LUMINOUSINTENSITYUNIT,
			ENUM_MAGNETICFLUXDENSITYUNIT,
			ENUM_MAGNETICFLUXUNIT,
			ENUM_MASSUNIT,
			ENUM_PLANEANGLEUNIT,
			ENUM_POWERUNIT,
			ENUM_PRESSUREUNIT,
			ENUM_RADIOACTIVITYUNIT,
			ENUM_SOLIDANGLEUNIT,
			ENUM_THERMODYNAMICTEMPERATUREUNIT,
			ENUM_TIMEUNIT,
			ENUM_VOLUMEUNIT,
			ENUM_USERDEFINED
		};

		IfcUnitEnum() = default;
		IfcUnitEnum( IfcUnitEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1197507443; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcUnitEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcUnitEnumEnum m_enum;
	};
}
