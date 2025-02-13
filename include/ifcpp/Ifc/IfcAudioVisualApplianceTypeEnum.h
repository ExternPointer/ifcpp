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
	// TYPE IfcAudioVisualApplianceTypeEnum = ENUMERATION OF	(AMPLIFIER	,CAMERA	,COMMUNICATIONTERMINAL	,DISPLAY	,MICROPHONE	,PLAYER	,PROJECTOR	,RECEIVER	,RECORDINGEQUIPMENT	,SPEAKER	,SWITCHER	,TELEPHONE	,TUNER	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcAudioVisualApplianceTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcAudioVisualApplianceTypeEnumEnum
		{
			ENUM_AMPLIFIER,
			ENUM_CAMERA,
			ENUM_COMMUNICATIONTERMINAL,
			ENUM_DISPLAY,
			ENUM_MICROPHONE,
			ENUM_PLAYER,
			ENUM_PROJECTOR,
			ENUM_RECEIVER,
			ENUM_RECORDINGEQUIPMENT,
			ENUM_SPEAKER,
			ENUM_SWITCHER,
			ENUM_TELEPHONE,
			ENUM_TUNER,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcAudioVisualApplianceTypeEnum() = default;
		IfcAudioVisualApplianceTypeEnum( IfcAudioVisualApplianceTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 2981638260; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcAudioVisualApplianceTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcAudioVisualApplianceTypeEnumEnum m_enum;
	};
}
