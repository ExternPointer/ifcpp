/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcSolidAngleMeasure.h"

// TYPE IfcSolidAngleMeasure = REAL;
IFC4X3::IfcSolidAngleMeasure::IfcSolidAngleMeasure( double value ) { m_value = value; }
void IFC4X3::IfcSolidAngleMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCSOLIDANGLEMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcSolidAngleMeasure> IFC4X3::IfcSolidAngleMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcSolidAngleMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcSolidAngleMeasure>(); }
	shared_ptr<IfcSolidAngleMeasure> type_object( new IfcSolidAngleMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}
