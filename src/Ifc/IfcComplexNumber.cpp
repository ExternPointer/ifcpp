/* Code generated by IfcQuery EXPRESS generator, www.ifcquery.com */

#include <sstream>
#include <limits>
#include <map>
#include "ifcpp/Reader/ReaderUtil.h"
#include "ifcpp/Writer/WriterUtil.h"
#include "ifcpp/Model/BasicTypes.h"
#include "ifcpp/Model/BuildingException.h"
#include "ifcpp/Ifc/IfcMeasureValue.h"
#include "ifcpp/Ifc/IfcComplexNumber.h"

// TYPE IfcComplexNumber = ARRAY [1:2] OF REAL;
void IFC4X3::IfcComplexNumber::getStepParameter( std::stringstream& stream, bool is_select_type ) const
{
	if( is_select_type ) { stream << "IFCCOMPLEXNUMBER("; }
	writeRealList( stream, m_vec );
	if( is_select_type ) { stream << ")"; }
}
shared_ptr<IFC4X3::IfcComplexNumber> IFC4X3::IfcComplexNumber::createObjectFromSTEP( const std::string& arg, const std::map<int,shared_ptr<BuildingEntity> >& map, std::stringstream& errorStream )
{
	if( arg.compare( "$" ) == 0 ) { return shared_ptr<IfcComplexNumber>(); }
	if( arg.compare( "*" ) == 0 ) { return shared_ptr<IfcComplexNumber>(); }
	shared_ptr<IfcComplexNumber> type_object( new IfcComplexNumber() );
	readRealList( arg, type_object->m_vec );
	return type_object;
}