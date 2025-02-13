/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#pragma once
#include <iostream>
#include <sstream>
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingObject.h"
#include "IfcValue.h"

namespace IFC4X3
{
	// TYPE IfcMeasureValue = SELECT	(IfcAmountOfSubstanceMeasure	,IfcAreaMeasure	,IfcComplexNumber	,IfcContextDependentMeasure	,IfcCountMeasure	,IfcDescriptiveMeasure	,IfcElectricCurrentMeasure	,IfcLengthMeasure	,IfcLuminousIntensityMeasure	,IfcMassMeasure	,IfcNonNegativeLengthMeasure	,IfcNormalisedRatioMeasure	,IfcNumericMeasure	,IfcParameterValue	,IfcPlaneAngleMeasure	,IfcPositiveLengthMeasure	,IfcPositivePlaneAngleMeasure	,IfcPositiveRatioMeasure	,IfcRatioMeasure	,IfcSolidAngleMeasure	,IfcThermodynamicTemperatureMeasure	,IfcTimeMeasure	,IfcVolumeMeasure);
	class IFCQUERY_EXPORT IfcMeasureValue : public IfcValue
	{
	public:
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const = 0;
		static shared_ptr<IfcMeasureValue> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
	};
}

