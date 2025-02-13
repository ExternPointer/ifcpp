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
	// TYPE IfcPermeableCoveringOperationEnum = ENUMERATION OF	(GRILL	,LOUVER	,SCREEN	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcPermeableCoveringOperationEnum : virtual public BuildingObject
	{
	public:
		enum IfcPermeableCoveringOperationEnumEnum
		{
			ENUM_GRILL,
			ENUM_LOUVER,
			ENUM_SCREEN,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcPermeableCoveringOperationEnum() = default;
		IfcPermeableCoveringOperationEnum( IfcPermeableCoveringOperationEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3446698506; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPermeableCoveringOperationEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcPermeableCoveringOperationEnumEnum m_enum;
	};
}
