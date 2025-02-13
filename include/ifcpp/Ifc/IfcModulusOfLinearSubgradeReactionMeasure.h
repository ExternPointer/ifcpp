/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <vector>
#include <map>
#include <sstream>
#include <string>
#include "ifcpp/Model/GlobalDefines.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcDerivedMeasureValue.h"
#include "IfcModulusOfTranslationalSubgradeReactionSelect.h"

namespace IFC4X3
{
	// TYPE IfcModulusOfLinearSubgradeReactionMeasure = REAL;
	class IFCQUERY_EXPORT IfcModulusOfLinearSubgradeReactionMeasure : public IfcDerivedMeasureValue, public IfcModulusOfTranslationalSubgradeReactionSelect
	{
	public:
		IfcModulusOfLinearSubgradeReactionMeasure() = default;
		IfcModulusOfLinearSubgradeReactionMeasure( double value );
		virtual uint32_t classID() const { return 2173214787; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcModulusOfLinearSubgradeReactionMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		double m_value;
	};
}
