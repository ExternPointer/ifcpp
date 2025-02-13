/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcBendingParameterSelect.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcSizeSelect.h"
#include "ifcpp/Ifc/IfcLengthMeasure.h"

// TYPE IfcLengthMeasure = REAL;
IFC4X3::IfcLengthMeasure::IfcLengthMeasure( double value ) { m_value = value; }
void IFC4X3::IfcLengthMeasure::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCLENGTHMEASURE("; }
	appendRealWithoutTrailingZeros( stream, m_value );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcLengthMeasure> IFC4X3::IfcLengthMeasure::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcLengthMeasure>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcLengthMeasure>(); }
	shared_ptr<IfcLengthMeasure> type_object( new IfcLengthMeasure() );
	readReal( arg, type_object->m_value );
	return type_object;
}
