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
	// TYPE IfcBearingTypeDisplacementEnum = ENUMERATION OF	(FIXED_MOVEMENT	,FREE_MOVEMENT	,GUIDED_LONGITUDINAL	,GUIDED_TRANSVERSAL	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcBearingTypeDisplacementEnum : virtual public BuildingObject
	{
	public:
		enum IfcBearingTypeDisplacementEnumEnum
		{
			ENUM_FIXED_MOVEMENT,
			ENUM_FREE_MOVEMENT,
			ENUM_GUIDED_LONGITUDINAL,
			ENUM_GUIDED_TRANSVERSAL,
			ENUM_NOTDEFINED
		};

		IfcBearingTypeDisplacementEnum() = default;
		IfcBearingTypeDisplacementEnum( IfcBearingTypeDisplacementEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3397575177; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcBearingTypeDisplacementEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcBearingTypeDisplacementEnumEnum m_enum;
	};
}
