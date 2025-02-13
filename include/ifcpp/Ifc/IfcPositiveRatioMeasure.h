/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcRatioMeasure.h"

namespace IFC4X3
{
	// TYPE IfcPositiveRatioMeasure = IfcRatioMeasure;
	class IFCQUERY_EXPORT IfcPositiveRatioMeasure : public IfcRatioMeasure
	{
	public:
		IfcPositiveRatioMeasure() = default;
		IfcPositiveRatioMeasure( double value ) { m_value = value; }
		virtual uint32_t classID() const { return 1245737093; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcPositiveRatioMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
	};
}
