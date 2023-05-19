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
	// TYPE IfcBSplineCurveForm = ENUMERATION OF	(CIRCULAR_ARC	,ELLIPTIC_ARC	,HYPERBOLIC_ARC	,PARABOLIC_ARC	,POLYLINE_FORM	,UNSPECIFIED);
	class IFCQUERY_EXPORT IfcBSplineCurveForm : virtual public BuildingObject
	{
	public:
		enum IfcBSplineCurveFormEnum
		{
			ENUM_CIRCULAR_ARC,
			ENUM_ELLIPTIC_ARC,
			ENUM_HYPERBOLIC_ARC,
			ENUM_PARABOLIC_ARC,
			ENUM_POLYLINE_FORM,
			ENUM_UNSPECIFIED
		};

		IfcBSplineCurveForm() = default;
		IfcBSplineCurveForm( IfcBSplineCurveFormEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 3573632694; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcBSplineCurveForm> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcBSplineCurveFormEnum m_enum;
	};
}
