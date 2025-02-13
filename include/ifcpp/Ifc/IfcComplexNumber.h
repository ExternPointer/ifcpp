/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcMeasureValue.h"

namespace IFC4X3
{
	// TYPE IfcComplexNumber = ARRAY [1:2] OF REAL;
	class IFCQUERY_EXPORT IfcComplexNumber : public IfcMeasureValue
	{
	public:
		IfcComplexNumber() = default;
		virtual uint32_t classID() const { return 2991860651; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcComplexNumber> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		std::vector<double> m_vec;
	};
}
