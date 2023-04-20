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

namespace IFC4X3
{
	// TYPE IfcSectionalAreaIntegralMeasure = REAL;
	class IFCQUERY_EXPORT IfcSectionalAreaIntegralMeasure : public IfcDerivedMeasureValue
	{
	public:
		IfcSectionalAreaIntegralMeasure() = default;
		IfcSectionalAreaIntegralMeasure( double value );
		virtual uint32_t classID() const { return 2190458107; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcSectionalAreaIntegralMeasure> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		double m_value;
	};
}
