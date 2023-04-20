/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcDerivedMeasureValue.h"
#include "ifcpp/Ifc/IfcHeatingValueMeasure.h"

// TYPE IfcHeatingValueMeasure = REAL;
IFC4X3::IfcHeatingValueMeasure::IfcHeatingValueMeasure( double value ) { m_value = value; }
void IFC4X3::IfcHeatingValueMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCHEATINGVALUEMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcHeatingValueMeasure> IFC4X3::IfcHeatingValueMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcHeatingValueMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcHeatingValueMeasure>(); }
	shared_ptr<IfcHeatingValueMeasure> type_object( new IfcHeatingValueMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}