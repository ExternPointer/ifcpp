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
	// TYPE IfcOutletTypeEnum = ENUMERATION OF	(AUDIOVISUALOUTLET	,COMMUNICATIONSOUTLET	,DATAOUTLET	,POWEROUTLET	,TELEPHONEOUTLET	,USERDEFINED	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcOutletTypeEnum : virtual public BuildingObject
	{
	public:
		enum IfcOutletTypeEnumEnum
		{
			ENUM_AUDIOVISUALOUTLET,
			ENUM_COMMUNICATIONSOUTLET,
			ENUM_DATAOUTLET,
			ENUM_POWEROUTLET,
			ENUM_TELEPHONEOUTLET,
			ENUM_USERDEFINED,
			ENUM_NOTDEFINED
		};

		IfcOutletTypeEnum() = default;
		IfcOutletTypeEnum( IfcOutletTypeEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 103775553; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcOutletTypeEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcOutletTypeEnumEnum m_enum;
	};
}
