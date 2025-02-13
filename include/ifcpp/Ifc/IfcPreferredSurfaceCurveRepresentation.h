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
	// TYPE IfcPreferredSurfaceCurveRepresentation = ENUMERATION OF	(CURVE3D	,PCURVE_S1	,PCURVE_S2);
	class IFCQUERY_EXPORT IfcPreferredSurfaceCurveRepresentation : virtual public BuildingObject
	{
	public:
		enum IfcPreferredSurfaceCurveRepresentationEnum
		{
			ENUM_CURVE3D,
			ENUM_PCURVE_S1,
			ENUM_PCURVE_S2
		};

		IfcPreferredSurfaceCurveRepresentation() = default;
		IfcPreferredSurfaceCurveRepresentation( IfcPreferredSurfaceCurveRepresentationEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 960210175; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPreferredSurfaceCurveRepresentation> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcPreferredSurfaceCurveRepresentationEnum m_enum;
	};
}
