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
	// TYPE IfcWallTypeEnum = ENUMERATION OF	(ELEMENTEDWALL	,MOVABLE	,PARAPET	,PARTITIONING	,PLUMBINGWALL	,POLYGONAL	,RETAININGWALL	,SHEAR	,SOLIDWALL	,STANDARD	,WAVEWALL	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcWallTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcWallTypeEnumEnum
		{
			ENUM_ELEMENTEDWALL,
			ENUM_MOVABLE,
			ENUM_PARAPET,
			ENUM_PARTITIONING,
			ENUM_PLUMBINGWALL,
			ENUM_POLYGONAL,
			ENUM_RETAININGWALL,
			ENUM_SHEAR,
			ENUM_SOLIDWALL,
			ENUM_STANDARD,
			ENUM_WAVEWALL,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcWallTypeEnum() = default;
		IfcWallTypeEnum( IfcWallTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3551551017; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcWallTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcWallTypeEnumEnum m_enum;
	};
}
