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
	// TYPE IfcStructuralCurveMemberTypeEnum = ENUMERATION OF	(CABLE	,COMPRESSION_MEMBER	,PIN_JOINED_MEMBER	,RIGID_JOINED_MEMBER	,TENSION_MEMBER	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcStructuralCurveMemberTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcStructuralCurveMemberTypeEnumEnum
		{
			ENUM_CABLE,
			ENUM_COMPRESSION_MEMBER,
			ENUM_PIN_JOINED_MEMBER,
			ENUM_RIGID_JOINED_MEMBER,
			ENUM_TENSION_MEMBER,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcStructuralCurveMemberTypeEnum() = default;
		IfcStructuralCurveMemberTypeEnum( IfcStructuralCurveMemberTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 2680421541; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcStructuralCurveMemberTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcStructuralCurveMemberTypeEnumEnum m_enum;
	};
}
