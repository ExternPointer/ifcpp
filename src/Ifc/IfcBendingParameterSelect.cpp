/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */
#include <map>
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"
#include "ifcpp/Ifc/IfcPlaneAngleMeasure.h"
#include "ifcpp/Ifc/IfcBendingParameterSelect.h"

// TYPE IfcBendingParameterSelect = SELECT	(IfcLengthMeasure	,IfcPlaneAngleMeasure);
shared_ptr<IFC4X3::IfcBendingParameterSelect> IFC4X3::IfcBendingParameterSelect::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.empty() ){ return shared_ptr<IfcBendingParameterSelect>(); }
	if( arg.compare("$")==0 )
	{
		return shared_ptr<IfcBendingParameterSelect>();
	}
	if( arg.compare("*")==0 )
	{
		return shared_ptr<IfcBendingParameterSelect>();
	}
	shared_ptr<IfcBendingParameterSelect> result_object;
	readSelectType( arg, result_object, map, errorStream );
	return result_object;
}
