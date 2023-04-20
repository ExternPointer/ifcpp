/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcVolumetricFlowRateMeasure.h"

// TYPE IfcVolumetricFlowRateMeasure = REAL;
IFC4X3::IfcVolumetricFlowRateMeasure::IfcVolumetricFlowRateMeasure( double value ) { m_value = value; }
void IFC4X3::IfcVolumetricFlowRateMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCVOLUMETRICFLOWRATEMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcVolumetricFlowRateMeasure> IFC4X3::IfcVolumetricFlowRateMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcVolumetricFlowRateMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcVolumetricFlowRateMeasure>(); }
	shared_ptr<IfcVolumetricFlowRateMeasure> type_object( new IfcVolumetricFlowRateMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}