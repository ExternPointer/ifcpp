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
	// TYPE IfcDistributionSystemEnum = ENUMERATION OF	(AIRCONDITIONING	,AUDIOVISUAL	,CATENARY_SYSTEM	,CHEMICAL	,CHILLEDWATER	,COMMUNICATION	,COMPRESSEDAIR	,CONDENSERWATER	,CONTROL	,CONVEYING	,DATA	,DISPOSAL	,DOMESTICCOLDWATER	,DOMESTICHOTWATER	,DRAINAGE	,EARTHING	,ELECTRICAL	,ELECTROACOUSTIC	,EXHAUST	,FIREPROTECTION	,FIXEDTRANSMISSIONNETWORK	,FUEL	,GAS	,HAZARDOUS	,HEATING	,LIGHTING	,LIGHTNINGPROTECTION	,MOBILENETWORK	,MONITORINGSYSTEM	,MUNICIPALSOLIDWASTE	,OIL	,OPERATIONAL	,OPERATIONALTELEPHONYSYSTEM	,OVERHEAD_CONTACTLINE_SYSTEM	,POWERGENERATION	,RAINWATER	,REFRIGERATION	,RETURN_CIRCUIT	,SECURITY	,SEWAGE	,SIGNAL	,STORMWATER	,TELEPHONE	,TV	,VACUUM	,VENT	,VENTILATION	,WASTEWATER	,WATERSUPPLY	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcDistributionSystemEnum : virtual public BuildingObject
	{
	public:
		enum IfcDistributionSystemEnumEnum
		{
			ENUM_AIRCONDITIONING,
			ENUM_AUDIOVISUAL,
			ENUM_CATENARY_SYSTEM,
			ENUM_CHEMICAL,
			ENUM_CHILLEDWATER,
			ENUM_COMMUNICATION,
			ENUM_COMPRESSEDAIR,
			ENUM_CONDENSERWATER,
			ENUM_CONTROL,
			ENUM_CONVEYING,
			ENUM_DATA,
			ENUM_DISPOSAL,
			ENUM_DOMESTICCOLDWATER,
			ENUM_DOMESTICHOTWATER,
			ENUM_DRAINAGE,
			ENUM_EARTHING,
			ENUM_ELECTRICAL,
			ENUM_ELECTROACOUSTIC,
			ENUM_EXHAUST,
			ENUM_FIREPROTECTION,
			ENUM_FIXEDTRANSMISSIONNETWORK,
			ENUM_FUEL,
			ENUM_GAS,
			ENUM_HAZARDOUS,
			ENUM_HEATING,
			ENUM_LIGHTING,
			ENUM_LIGHTNINGPROTECTION,
			ENUM_MOBILENETWORK,
			ENUM_MONITORINGSYSTEM,
			ENUM_MUNICIPALSOLIDWASTE,
			ENUM_OIL,
			ENUM_OPERATIONAL,
			ENUM_OPERATIONALTELEPHONYSYSTEM,
			ENUM_OVERHEAD_CONTACTLINE_SYSTEM,
			ENUM_POWERGENERATION,
			ENUM_RAINWATER,
			ENUM_REFRIGERATION,
			ENUM_RETURN_CIRCUIT,
			ENUM_SECURITY,
			ENUM_SEWAGE,
			ENUM_SIGNAL,
			ENUM_STORMWATER,
			ENUM_TELEPHONE,
			ENUM_TV,
			ENUM_VACUUM,
			ENUM_VENT,
			ENUM_VENTILATION,
			ENUM_WASTEWATER,
			ENUM_WATERSUPPLY,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcDistributionSystemEnum() = default;
		IfcDistributionSystemEnum( IfcDistributionSystemEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 922449830; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcDistributionSystemEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcDistributionSystemEnumEnum m_enum;
	};
}
