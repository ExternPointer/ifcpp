/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"

namespace IFC4X3
{
	// TYPE IfcTrimmingPreference = ENUMERATION OF	(CARTESIAN	,PARAMETER	,UNSPECIFIED);
	class IFCQUERY_EXPORT IfcTrimmingPreference : virtual public BuildingObject
	{
	public:
		enum IfcTrimmingPreferenceEnum
		{
			ENUM_CARTESIAN,
			ENUM_PARAMETER,
			ENUM_UNSPECIFIED
		};

		IfcTrimmingPreference() = default;
		IfcTrimmingPreference( IfcTrimmingPreferenceEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3407053508; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcTrimmingPreference> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcTrimmingPreferenceEnum m_enum;
	};
}

