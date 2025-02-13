/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcNumericMeasure.h"

// TYPE IfcNumericMeasure = NUMBER;
IFC4X3::IfcNumericMeasure::IfcNumericMeasure( int value ) { m_value = value; }
void IFC4X3::IfcNumericMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCNUMERICMEASURE("; }
	stream << m_value;
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcNumericMeasure> IFC4X3::IfcNumericMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcNumericMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcNumericMeasure>(); }
	shared_ptr<IfcNumericMeasure> type_object( new IfcNumericMeasure() );
	readInteger( arg, type_object->m_value );
	return type_object;
}
