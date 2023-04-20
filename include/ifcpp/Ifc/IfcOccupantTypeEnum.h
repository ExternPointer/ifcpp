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
	// TYPE IfcOccupantTypeEnum = ENUMERATION OF	(ASSIGNEE	,ASSIGNOR	,LESSEE	,LESSOR	,LETTINGAGENT	,OWNER	,TENANT	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcOccupantTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcOccupantTypeEnumEnum
		{
			ENUM_ASSIGNEE,
			ENUM_ASSIGNOR,
			ENUM_LESSEE,
			ENUM_LESSOR,
			ENUM_LETTINGAGENT,
			ENUM_OWNER,
			ENUM_TENANT,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcOccupantTypeEnum() = default;
		IfcOccupantTypeEnum( IfcOccupantTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3349296550; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcOccupantTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcOccupantTypeEnumEnum m_enum;
	};
}
