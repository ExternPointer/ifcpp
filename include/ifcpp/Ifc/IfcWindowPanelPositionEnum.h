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
	// TYPE IfcWindowPanelPositionEnum = ENUMERATION OF	(BOTTOM	,LEFT	,MIDDLE	,RIGHT	,TOP	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcWindowPanelPositionEnum : virtual public BuildingObject
	{
	public:
		enum IfcWindowPanelPositionEnumEnum
		{
			ENUM_BOTTOM,
			ENUM_LEFT,
			ENUM_MIDDLE,
			ENUM_RIGHT,
			ENUM_TOP,
			ENUM_NOTDEFINED
		};

		IfcWindowPanelPositionEnum() = default;
		IfcWindowPanelPositionEnum( IfcWindowPanelPositionEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 278839091; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcWindowPanelPositionEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcWindowPanelPositionEnumEnum m_enum;
	};
}
