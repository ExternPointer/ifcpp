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
	// TYPE IfcDerivedUnitEnum = ENUMERATION OF	(ACCELERATIONUNIT	,ANGULARVELOCITYUNIT	,AREADENSITYUNIT	,COMPOUNDPLANEANGLEUNIT	,CURVATUREUNIT	,DYNAMICVISCOSITYUNIT	,HEATFLUXDENSITYUNIT	,HEATINGVALUEUNIT	,INTEGERCOUNTRATEUNIT	,IONCONCENTRATIONUNIT	,ISOTHERMALMOISTURECAPACITYUNIT	,KINEMATICVISCOSITYUNIT	,LINEARFORCEUNIT	,LINEARMOMENTUNIT	,LINEARSTIFFNESSUNIT	,LINEARVELOCITYUNIT	,LUMINOUSINTENSITYDISTRIBUTIONUNIT	,MASSDENSITYUNIT	,MASSFLOWRATEUNIT	,MASSPERLENGTHUNIT	,MODULUSOFELASTICITYUNIT	,MODULUSOFLINEARSUBGRADEREACTIONUNIT	,MODULUSOFROTATIONALSUBGRADEREACTIONUNIT	,MODULUSOFSUBGRADEREACTIONUNIT	,MOISTUREDIFFUSIVITYUNIT	,MOLECULARWEIGHTUNIT	,MOMENTOFINERTIAUNIT	,PHUNIT	,PLANARFORCEUNIT	,ROTATIONALFREQUENCYUNIT	,ROTATIONALMASSUNIT	,ROTATIONALSTIFFNESSUNIT	,SECTIONAREAINTEGRALUNIT	,SECTIONMODULUSUNIT	,SHEARMODULUSUNIT	,SOUNDPOWERLEVELUNIT	,SOUNDPOWERUNIT	,SOUNDPRESSURELEVELUNIT	,SOUNDPRESSUREUNIT	,SPECIFICHEATCAPACITYUNIT	,TEMPERATUREGRADIENTUNIT	,TEMPERATURERATEOFCHANGEUNIT	,THERMALADMITTANCEUNIT	,THERMALCONDUCTANCEUNIT	,THERMALEXPANSIONCOEFFICIENTUNIT	,THERMALRESISTANCEUNIT	,THERMALTRANSMITTANCEUNIT	,TORQUEUNIT	,VAPORPERMEABILITYUNIT	,VOLUMETRICFLOWRATEUNIT	,WARPINGCONSTANTUNIT	,WARPINGMOMENTUNIT	,USERDEFINED);
	class IFCQUERY_EXPORT IfcDerivedUnitEnum : virtual public BuildingObject
	{
	public:
		enum IfcDerivedUnitEnumEnum
		{
			ENUM_ACCELERATIONUNIT,
			ENUM_ANGULARVELOCITYUNIT,
			ENUM_AREADENSITYUNIT,
			ENUM_COMPOUNDPLANEANGLEUNIT,
			ENUM_CURVATUREUNIT,
			ENUM_DYNAMICVISCOSITYUNIT,
			ENUM_HEATFLUXDENSITYUNIT,
			ENUM_HEATINGVALUEUNIT,
			ENUM_INTEGERCOUNTRATEUNIT,
			ENUM_IONCONCENTRATIONUNIT,
			ENUM_ISOTHERMALMOISTURECAPACITYUNIT,
			ENUM_KINEMATICVISCOSITYUNIT,
			ENUM_LINEARFORCEUNIT,
			ENUM_LINEARMOMENTUNIT,
			ENUM_LINEARSTIFFNESSUNIT,
			ENUM_LINEARVELOCITYUNIT,
			ENUM_LUMINOUSINTENSITYDISTRIBUTIONUNIT,
			ENUM_MASSDENSITYUNIT,
			ENUM_MASSFLOWRATEUNIT,
			ENUM_MASSPERLENGTHUNIT,
			ENUM_MODULUSOFELASTICITYUNIT,
			ENUM_MODULUSOFLINEARSUBGRADEREACTIONUNIT,
			ENUM_MODULUSOFROTATIONALSUBGRADEREACTIONUNIT,
			ENUM_MODULUSOFSUBGRADEREACTIONUNIT,
			ENUM_MOISTUREDIFFUSIVITYUNIT,
			ENUM_MOLECULARWEIGHTUNIT,
			ENUM_MOMENTOFINERTIAUNIT,
			ENUM_PHUNIT,
			ENUM_PLANARFORCEUNIT,
			ENUM_ROTATIONALFREQUENCYUNIT,
			ENUM_ROTATIONALMASSUNIT,
			ENUM_ROTATIONALSTIFFNESSUNIT,
			ENUM_SECTIONAREAINTEGRALUNIT,
			ENUM_SECTIONMODULUSUNIT,
			ENUM_SHEARMODULUSUNIT,
			ENUM_SOUNDPOWERLEVELUNIT,
			ENUM_SOUNDPOWERUNIT,
			ENUM_SOUNDPRESSURELEVELUNIT,
			ENUM_SOUNDPRESSUREUNIT,
			ENUM_SPECIFICHEATCAPACITYUNIT,
			ENUM_TEMPERATUREGRADIENTUNIT,
			ENUM_TEMPERATURERATEOFCHANGEUNIT,
			ENUM_THERMALADMITTANCEUNIT,
			ENUM_THERMALCONDUCTANCEUNIT,
			ENUM_THERMALEXPANSIONCOEFFICIENTUNIT,
			ENUM_THERMALRESISTANCEUNIT,
			ENUM_THERMALTRANSMITTANCEUNIT,
			ENUM_TORQUEUNIT,
			ENUM_VAPORPERMEABILITYUNIT,
			ENUM_VOLUMETRICFLOWRATEUNIT,
			ENUM_WARPINGCONSTANTUNIT,
			ENUM_WARPINGMOMENTUNIT,
			ENUM_USERDEFINED
		};

		IfcDerivedUnitEnum() = default;
		IfcDerivedUnitEnum( IfcDerivedUnitEnumEnum e ) { m_enum = e; }
		virtual uint32_t classID() const { return 124742581; }
		virtual void getStepParameter( std::stringstream& stream, bool is_select_type = false ) const;
		static shared_ptr<IfcDerivedUnitEnum> createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream );
		IfcDerivedUnitEnumEnum m_enum;
	};
}
