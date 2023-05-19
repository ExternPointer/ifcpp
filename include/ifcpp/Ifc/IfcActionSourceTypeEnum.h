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
	// TYPE IfcActionSourceTypeEnum = ENUMERATION OF	(BRAKES	,BUOYANCY	,COMPLETION_G1	,CREEP	,CURRENT	,DEAD_LOAD_G	,EARTHQUAKE_E	,ERECTION	,FIRE	,ICE	,IMPACT	,IMPULSE	,LACK_OF_FIT	,LIVE_LOAD_Q	,PRESTRESSING_P	,PROPPING	,RAIN	,SETTLEMENT_U	,SHRINKAGE	,SNOW_S	,SYSTEM_IMPERFECTION	,TEMPERATURE_T	,TRANSPORT	,WAVE	,WIND_W	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcActionSourceTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcActionSourceTypeEnumEnum
		{
			ENUM_BRAKES,
			ENUM_BUOYANCY,
			ENUM_COMPLETION_G1,
			ENUM_CREEP,
			ENUM_CURRENT,
			ENUM_DEAD_LOAD_G,
			ENUM_EARTHQUAKE_E,
			ENUM_ERECTION,
			ENUM_FIRE,
			ENUM_ICE,
			ENUM_IMPACT,
			ENUM_IMPULSE,
			ENUM_LACK_OF_FIT,
			ENUM_LIVE_LOAD_Q,
			ENUM_PRESTRESSING_P,
			ENUM_PROPPING,
			ENUM_RAIN,
			ENUM_SETTLEMENT_U,
			ENUM_SHRINKAGE,
			ENUM_SNOW_S,
			ENUM_SYSTEM_IMPERFECTION,
			ENUM_TEMPERATURE_T,
			ENUM_TRANSPORT,
			ENUM_WAVE,
			ENUM_WIND_W,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcActionSourceTypeEnum() = default;
		IfcActionSourceTypeEnum( IfcActionSourceTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 126693432; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcActionSourceTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcActionSourceTypeEnumEnum m_enum;
	};
}
