/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcPlaneAngleMeasure.h"

namespace IFC4X3
{
	// TYPE IfcPositivePlaneAngleMeasure = IfcPlaneAngleMeasure;
	class IFCQUERY_EXPORT IfcPositivePlaneAngleMeasure : public IfcPlaneAngleMeasure
	{
	public:
		IfcPositivePlaneAngleMeasure() = default;
		IfcPositivePlaneAngleMeasure( double value ) { m_value = value; }
		virtual uint32_t classID() const { return 3054510233; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPositivePlaneAngleMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
	};
}
