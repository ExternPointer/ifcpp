/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcTrackElementTypeEnum = ENUMERATION OF	(BLOCKINGDEVICE	,DERAILER	,FROG	,HALF_SET_OF_BLADES	,SLEEPER	,SPEEDREGULATOR	,TRACKENDOFALIGNMENT	,VEHICLESTOP	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcTrackElementTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcTrackElementTypeEnumEnum
		{
			ENUM_BLOCKINGDEVICE,
			ENUM_DERAILER,
			ENUM_FROG,
			ENUM_HALF_SET_OF_BLADES,
			ENUM_SLEEPER,
			ENUM_SPEEDREGULATOR,
			ENUM_TRACKENDOFALIGNMENT,
			ENUM_VEHICLESTOP,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcTrackElementTypeEnum() = default;
		IfcTrackElementTypeEnum( IfcTrackElementTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 859079163; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcTrackElementTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcTrackElementTypeEnumEnum m_enum;
	};
}

