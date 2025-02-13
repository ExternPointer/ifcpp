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
	// TYPE IfcPermitTypeEnum = ENUMERATION OF	(ACCESS	,BUILDING	,WORK	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcPermitTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcPermitTypeEnumEnum
		{
			ENUM_ACCESS,
			ENUM_BUILDING,
			ENUM_WORK,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcPermitTypeEnum() = default;
		IfcPermitTypeEnum( IfcPermitTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3061959087; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPermitTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcPermitTypeEnumEnum m_enum;
	};
}
