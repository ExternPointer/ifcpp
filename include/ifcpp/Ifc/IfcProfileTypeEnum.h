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
	// TYPE IfcProfileTypeEnum = ENUMERATION OF	(AREA	,CURVE);
	class IFCQUERY_EXPORT IfcProfileTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcProfileTypeEnumEnum
		{
			ENUM_AREA,
			ENUM_CURVE
		};

		IfcProfileTypeEnum() = default;
		IfcProfileTypeEnum( IfcProfileTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 2321227483; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcProfileTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcProfileTypeEnumEnum m_enum;
	};
}
