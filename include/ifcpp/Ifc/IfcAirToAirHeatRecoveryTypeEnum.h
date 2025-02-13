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
	// TYPE IfcAirToAirHeatRecoveryTypeEnum = ENUMERATION OF	(FIXEDPLATECOUNTERFLOWEXCHANGER	,FIXEDPLATECROSSFLOWEXCHANGER	,FIXEDPLATEPARALLELFLOWEXCHANGER	,HEATPIPE	,ROTARYWHEEL	,RUNAROUNDCOILLOOP	,THERMOSIPHONCOILTYPEHEATEXCHANGERS	,THERMOSIPHONSEALEDTUBEHEATEXCHANGERS	,TWINTOWERENTHALPYRECOVERYLOOPS	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcAirToAirHeatRecoveryTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcAirToAirHeatRecoveryTypeEnumEnum
		{
			ENUM_FIXEDPLATECOUNTERFLOWEXCHANGER,
			ENUM_FIXEDPLATECROSSFLOWEXCHANGER,
			ENUM_FIXEDPLATEPARALLELFLOWEXCHANGER,
			ENUM_HEATPIPE,
			ENUM_ROTARYWHEEL,
			ENUM_RUNAROUNDCOILLOOP,
			ENUM_THERMOSIPHONCOILTYPEHEATEXCHANGERS,
			ENUM_THERMOSIPHONSEALEDTUBEHEATEXCHANGERS,
			ENUM_TWINTOWERENTHALPYRECOVERYLOOPS,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcAirToAirHeatRecoveryTypeEnum() = default;
		IfcAirToAirHeatRecoveryTypeEnum( IfcAirToAirHeatRecoveryTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1797193231; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcAirToAirHeatRecoveryTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcAirToAirHeatRecoveryTypeEnumEnum m_enum;
	};
}
