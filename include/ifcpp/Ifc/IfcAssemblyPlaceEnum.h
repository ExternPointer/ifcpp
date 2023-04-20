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
	// TYPE IfcAssemblyPlaceEnum = ENUMERATION OF	(FACTORY	,SITE	,NOTDEFINED);
	class IFCQUERY_EXPORT IfcAssemblyPlaceEnum : virtual public BuildingObject
	{
	public:
		enum IfcAssemblyPlaceEnumEnum
		{
			ENUM_FACTORY,
			ENUM_SITE,
			ENUM_NOTDEFINED
		};

		IfcAssemblyPlaceEnum() = default;
		IfcAssemblyPlaceEnum( IfcAssemblyPlaceEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 1925676203; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcAssemblyPlaceEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcAssemblyPlaceEnumEnum m_enum;
	};
}
