/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/model/GlobalDefines.h"
#include "ifcpp/model/BasicTypes.h"
#include "ifcpp/model/BuildingObject.h"
#include "IfcMeasureValue.h"

namespace IFC4X3
{
	// TYPE IfcCountMeasure = INTEGER;
	class IFCQUERY_EXPORT IfcCountMeasure : public IfcMeasureValue
	{
	public:
		IfcCountMeasure() = default;
		IfcCountMeasure( int value );
		virtual uint32_t classID() const { return 1778710042; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcCountMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		int m_value;
	};
}

