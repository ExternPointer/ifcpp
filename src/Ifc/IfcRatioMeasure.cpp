/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcSizeSelect.h"
#include "ifcpp/Ifc/IfcTimeOrRatioSelect.h"
#include "ifcpp/Ifc/IfcRatioMeasure.h"

// TYPE IfcRatioMeasure = REAL;
IFC4X3::IfcRatioMeasure::IfcRatioMeasure( double value ) { m_value = value; }
void IFC4X3::IfcRatioMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCRATIOMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcRatioMeasure> IFC4X3::IfcRatioMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcRatioMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcRatioMeasure>(); }
	shared_ptr<IfcRatioMeasure> type_object( new IfcRatioMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}