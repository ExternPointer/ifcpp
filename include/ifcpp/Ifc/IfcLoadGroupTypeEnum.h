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
	// TYPE IfcLoadGroupTypeEnum = ENUMERATION OF	(LOAD_CASE	,LOAD_COMBINATION	,LOAD_GROUP	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcLoadGroupTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcLoadGroupTypeEnumEnum
		{
			ENUM_LOAD_CASE,
			ENUM_LOAD_COMBINATION,
			ENUM_LOAD_GROUP,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcLoadGroupTypeEnum() = default;
		IfcLoadGroupTypeEnum( IfcLoadGroupTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 2602792976; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcLoadGroupTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcLoadGroupTypeEnumEnum m_enum;
	};
}
