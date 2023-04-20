/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Ifc/IfcResourceObjectSelect.h"

// TYPE IfcResourceObjectSelect = SELECT	(IfcActorRole	,IfcAppliedValue	,IfcApproval	,IfcConstraint	,IfcContextDependentUnit	,IfcConversionBasedUnit	,IfcExternalInformation	,IfcExternalReference	,IfcMaterialDefinition	,IfcOrganization	,IfcPerson	,IfcPersonAndOrganization	,IfcPhysicalQuantity	,IfcProfileDef	,IfcPropertyAbstraction	,IfcShapeAspect	,IfcTimeSeries);
shared_ptr<IFC4X3::IfcResourceObjectSelect> IFC4X3::IfcResourceObjectSelect::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.empty() ){ return shared_ptr<IfcResourceObjectSelect>(); }
	if( arg.compare("$")==0 )
	{
		return shared_ptr<IfcResourceObjectSelect>();
	}
	if( arg.compare("*")==0 )
	{
		return shared_ptr<IfcResourceObjectSelect>();
	}
	shared_ptr<IfcResourceObjectSelect> result_object;
	readSelectType( arg, result_object, map, errorStream );
	return result_object;
}